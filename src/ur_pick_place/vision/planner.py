"""Compose vision targets into the existing pick/place recipe; preview, then execute.

The marker-derived pick pose replaces ``recipe.pick_pose`` (the force-search endpoint
and grasp reference) and an optional marker-derived place pose replaces
``recipe.place_pose``. Everything after that is the unchanged
``PickPlaceController`` sequence: home, approach, force descent, grip, lift, place.

Execution requires a preview made from the same inputs, no older than
``preview.max_age_s``; the fresh observation must reproduce the previewed targets
within ``preview.max_deviation_mm/deg``. A preview authorizes one execution.
"""
from dataclasses import replace
import hashlib
import math
from pathlib import Path
import secrets
import time

import numpy as np

from ..controller import PickPlaceController
from .errors import TargetValidationError, VisionError
from .grasp import grasp_for, target_pose
from .observe import Observer, RobotMover
from .robot_state import ControlPoseSource
from .targets import target_from_config
from .transforms import matrix_to_pose, pose_to_matrix, rotation_angle

PREVIEW_SCHEMA = 1


def file_sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class VisionJob:
    def __init__(self, recipe, config, calibration, *, inputs=None):
        self.recipe, self.config, self.calibration = recipe, config, calibration
        self.roles = ['pick'] + (['place'] if config.place is not None else [])
        self.models = {role: grasp_for(config, role) for role in self.roles}
        self.targets = {role: target_from_config(config.role(role).target, config.base_dir) for role in self.roles}
        self.inputs = dict(inputs or {}, calibration=calibration.sha256,
                           **{f'{role}_grasp': self.models[role].sha256 or self.models[role].source
                              for role in self.roles})

    @property
    def views(self):
        """Taught marker-relative camera views to reproduce during refine."""
        return {role: model.taught_view for role, model in self.models.items()
                if model.taught_refined and model.taught_view is not None}

    def compose(self, observations, tcp_offset, *, strict=True):
        """Targets from observations, validated; raises before anything can move."""
        poses, reports = {}, {}
        for role in self.roles:
            if role not in observations:
                raise VisionError(f'No {role} observation')
            if observations[role].role != role:
                raise VisionError(f'{role} observation is labeled {observations[role].role!r}; observe again')
            pose, report = target_pose(self.models[role], observations[role], self.config.role(role),
                                       calibration_sha256=self.calibration.sha256, tcp_offset=tcp_offset,
                                       target=self.targets[role], strict=strict)
            poses[role], reports[role] = pose, report
        pick = matrix_to_pose(poses['pick'])
        place = matrix_to_pose(poses['place']) if 'place' in poses else list(self.recipe.place_pose)
        recipe = replace(self.recipe, pick_pose=pick, place_pose=place)
        try:
            envelope = PickPlaceController(recipe).plan()
        except Exception as exc:
            raise TargetValidationError('Vision targets refused by the pick/place envelope', [str(exc)]) from exc
        if 'place' not in reports:
            reports['place'] = {'role': 'place', 'source': 'recipe place_pose', 'tcp_pose_m_rad': place}
        return {'recipe': recipe, 'pick_pose': pick, 'place_pose': place, 'targets': reports,
                'observations': {role: observations[role].summary() for role in self.roles},
                'envelope': envelope}

    @staticmethod
    def public(plan):
        return {key: value for key, value in plan.items() if key != 'recipe'}

    def preview_record(self, plan, *, now=None):
        return {'schema_version': PREVIEW_SCHEMA, 'type': 'vision_pick_preview', 'id': secrets.token_hex(8),
                'created_at': time.time() if now is None else now, 'robot_ip': self.recipe.robot_ip,
                'inputs': self.inputs, 'pick_pose': plan['pick_pose'], 'place_pose': plan['place_pose'],
                'consumed_at': None, 'plan': self.public(plan)}

    def check_preview(self, preview, *, now=None):
        """Static checks before any motion: same inputs, fresh, unused."""
        if not isinstance(preview, dict) or preview.get('type') != 'vision_pick_preview' \
                or preview.get('schema_version') != PREVIEW_SCHEMA:
            raise VisionError('Not a vision-pick preview; run vision-pick without --execute first')
        if preview.get('consumed_at') is not None:
            raise VisionError('This preview was already used for an execution; preview again')
        if preview.get('robot_ip') != self.recipe.robot_ip:
            raise VisionError('Preview was made for a different robot; preview again')
        if preview.get('inputs') != self.inputs:
            changed = sorted(k for k in set(self.inputs) | set(preview.get('inputs') or {})
                             if (preview.get('inputs') or {}).get(k) != self.inputs.get(k))
            raise VisionError(f'Inputs changed since the preview ({", ".join(changed)}); preview again')
        age = (time.time() if now is None else now) - float(preview.get('created_at', 0))
        if not 0 <= age <= self.config.preview.max_age_s:
            raise VisionError(f'Preview is {age:.0f} s old (limit {self.config.preview.max_age_s:g} s); preview again')

    def check_against_preview(self, preview, plan):
        limits, reasons = self.config.preview, []
        for key in ('pick_pose', 'place_pose'):
            a, b = pose_to_matrix(preview[key]), pose_to_matrix(plan[key])
            shift = float(np.linalg.norm(a[:3, 3] - b[:3, 3])) * 1000
            turn = math.degrees(rotation_angle(a, b))
            if shift > limits.max_deviation_mm or turn > limits.max_deviation_deg:
                reasons.append(f'{key} moved {shift:.2f} mm / {turn:.2f} deg since the preview '
                               f'(limit {limits.max_deviation_mm:g} mm / {limits.max_deviation_deg:g} deg)')
        if reasons:
            raise TargetValidationError('Fresh observation does not match the preview', reasons)

    def check_reachability(self, control, plan):
        """Controller checks for every envelope pose before handing over to the sequence."""
        reasons = []
        for name, pose in plan['envelope'].items():
            if control.isPoseWithinSafetyLimits(pose) is not True:
                reasons.append(f'{name} {pose} outside robot safety limits')
            elif control.getInverseKinematicsHasSolution(pose, list(self.recipe.home_joints)) is not True:
                reasons.append(f'{name} {pose} has no inverse-kinematics solution')
        if reasons:
            raise TargetValidationError('Vision targets are not reachable', reasons)

    def observe_all(self, camera, robot, on_step=None):
        """Execution-time observations with observe/refine motion through ``robot`` (RTDERobot)."""
        source = ControlPoseSource(robot.control, robot.receive)
        observer = Observer(self.config, self.calibration, camera, source, mover=RobotMover(robot), on_step=on_step,
                            views=self.views)
        observations = {role: observer.look(role, allow_motion=True) for role in self.roles}
        return observations, source.snapshot().tcp_offset

    def execute(self, preview, controller, camera, *, on_step=None, consume=None):
        """Observe, re-validate against the preview, then run the existing force-guided cycle once.

        ``consume(preview)`` persists that the preview has been used; it runs after the
        preview checks and before any motion, so a preview authorizes one attempt.
        """
        log = on_step or (lambda message: None)
        self.check_preview(preview)
        preview['consumed_at'] = time.time()
        if consume is not None:
            consume(preview)
        robot, gripper = controller.robot, controller.gripper
        gripper.require_ready()
        observations, tcp_offset = self.observe_all(camera, robot, log)
        plan = self.compose(observations, tcp_offset)
        self.check_against_preview(preview, plan)
        self.check_reachability(robot.control, plan)
        log(f'Vision targets confirmed; pick {plan["pick_pose"]}, place {plan["place_pose"]}')
        sequence = PickPlaceController(plan['recipe'], robot, gripper,
                                       on_step=lambda key, message: log(message))
        sequence.run(pick_pose=plan['pick_pose'], cycles=1)
        return plan
