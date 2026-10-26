"""Motion sequencing independent of RTDE, Qt, vision, and gripper implementation."""
from dataclasses import dataclass
import logging
import time

from .config import vector

logger = logging.getLogger(__name__)


class SequenceError(RuntimeError):
    pass


def shifted(pose, xyz_mm):
    result = list(pose)
    for axis, value in enumerate(xyz_mm):
        result[axis] += value / 1000.0
    return result


@dataclass
class ProbeResult:
    pose: list
    reason: str  # 'contact' or 'endpoint'


class PickPlaceController:
    """Inject robot/gripper backends; targets are TCP poses in the robot base.

    The caller owns backend connection lifetimes. Failures propagate without
    an automatic release, recovery move, or next cycle.
    """

    def __init__(self, recipe, robot=None, gripper=None):
        self.recipe, self.robot, self.gripper = recipe, robot, gripper

    def validate_pose(self, pose):
        pose = vector(pose, 6, 'TCP pose')
        for axis in range(3):
            if not self.recipe.workspace_min_m[axis] <= pose[axis] <= self.recipe.workspace_max_m[axis]:
                raise SequenceError(f'TCP pose outside workspace on axis {axis}: {pose}')
        return pose

    def plan(self, pick_pose=None):
        c = self.recipe
        pick = vector(c.pick_pose if pick_pose is None else pick_pose, 6, 'pick_pose')
        pick_approach = shifted(pick, [0, 0, c.pick_approach_mm])
        place_approach = shifted(c.place_pose, [0, 0, c.place_approach_mm])
        pick_floor = list(pick_approach)
        pick_floor[2] = max(pick[2], pick_approach[2] - c.pick_probe.max_descent_mm / 1000)
        place_floor = shifted(place_approach, [0, 0, -c.place_probe.max_descent_mm])
        # Extremes bound all possible higher-contact backoffs, offsets and lifts.
        grasp_low = shifted(pick_floor, c.grasp_offset_mm)
        grasp_high = shifted(shifted(pick_approach, [0, 0, c.pick_backoff_mm]), c.grasp_offset_mm)
        plan = dict(pick=pick, pick_approach=pick_approach, pick_floor=pick_floor,
                    grasp_lower_bound=grasp_low, grasp_upper_bound=grasp_high,
                    lift_lower_bound=shifted(grasp_low, [0, 0, c.lift_mm]),
                    lift_upper_bound=shifted(grasp_high, [0, 0, c.lift_mm]),
                    place_reference=list(c.place_pose), place_approach=place_approach,
                    place_floor=place_floor,
                    retract_lower_bound=shifted(place_floor, [0, 0, c.retract_mm]),
                    retract_upper_bound=shifted(place_approach, [0, 0, c.retract_mm]))
        for pose in plan.values():
            self.validate_pose(pose)
        return plan

    def run(self, *, pick_pose=None, cycles=1, cycle_wait_s=2.0):
        if type(cycles) is not int or cycles < 1:
            raise ValueError('cycles must be a positive integer')
        from .config import number
        number(cycle_wait_s, 'cycle_wait_s', low=0, high=3600)
        c = self.recipe
        c.gripper.require_live_ready()
        plan = self.plan(pick_pose)
        if self.robot is None or self.gripper is None:
            raise ValueError('Execution requires connected robot and gripper backends')
        # Check all possible extrema before the first move.
        self.robot.preflight(c.home_joints, list(plan.values()))
        for cycle in range(cycles):
            logger.info('Starting cycle %d/%d', cycle + 1, cycles)
            self.gripper.require_ready()
            logger.info('Moving home, opening gripper, approaching pick')
            self.robot.move_joints(c.home_joints)
            self.gripper.open()
            self._linear(plan['pick_approach'])
            with self.gripper.probe_guard() as guard:
                result = self.robot.probe(c.pick_probe, endpoint_z=plan['pick'][2], guard=guard)
            # probe() must stop force mode successfully before returning.
            measured = self.validate_pose(result.pose)
            if result.reason == 'endpoint' and measured[2] <= plan['pick'][2]:
                grasp = list(plan['pick'])
            elif result.reason == 'contact' and measured[2] > plan['pick'][2]:
                grasp = shifted(measured, [0, 0, c.pick_backoff_mm])
            else:
                raise SequenceError('Invalid pick stop condition; grip blocked')
            grasp = self.validate_pose(shifted(grasp, c.grasp_offset_mm))
            lift = self.validate_pose(shifted(grasp, [0, 0, c.lift_mm]))
            logger.info('Pick stopped on %s; moving to grasp pose %s', result.reason, grasp)
            self._linear(grasp, loaded=True)
            logger.info('Closing gripper and verifying grip')
            self.gripper.grip()
            self.gripper.require_holding()
            self._linear(lift, loaded=True)
            self.gripper.require_holding()
            self.robot.move_joints(c.home_joints)
            self.gripper.require_holding()
            logger.info('Approaching placement and probing for contact')
            self._linear(plan['place_approach'])
            placed = self.robot.probe(c.place_probe)
            if placed.reason != 'contact':
                raise SequenceError('Placement requires contact; release blocked')
            placed_pose = self.validate_pose(placed.pose)
            retract = self.validate_pose(shifted(placed_pose, [0, 0, c.retract_mm]))
            logger.info('Releasing at measured pose %s, then retracting', placed_pose)
            self.gripper.open()  # Includes release verification; failure blocks retract.
            self._linear(retract)
            if c.return_home:
                self.robot.move_joints(c.home_joints)
            logger.info('Completed cycle %d/%d', cycle + 1, cycles)
            if cycle + 1 < cycles:
                time.sleep(cycle_wait_s)
        return cycles

    def _linear(self, pose, loaded=False):
        self.robot.move_linear(self.validate_pose(pose), grasp_speed=loaded)
