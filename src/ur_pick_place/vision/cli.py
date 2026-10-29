"""Vision-guided pick/place: observe, teach-grasp, vision-pick.

observe       Detect a target and print its pose in the robot base frame. Read-only by
              default (RTDE receive + camera at the current pose); --execute moves to
              the observe joints and performs the optional refine view.
teach-grasp   Store the marker-to-TCP relationship from a saved observation and a
              taught TCP pose. Offline unless --from-robot reads the current TCP.
vision-pick   Preview (default): read-only observation, validated pick/place targets
              and the full approach/descent envelope, saved as a preview file.
              --execute --preview FILE re-observes, checks the result matches the
              preview, then runs the existing force-guided cycle once.
"""
import argparse
import json
import logging
from contextlib import contextmanager
import os
from pathlib import Path

from ..config import Recipe, vector
from ..poses import atomic_json

log = logging.getLogger('ur_pick_place.vision')


def _parser():
    parser = argparse.ArgumentParser(prog='ur-vision', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest='command', required=True)

    def common(sub, recipe_required=True):
        sub.add_argument('--config', type=Path, required=recipe_required, help='Pick/place recipe JSON')
        sub.add_argument('--vision', type=Path, required=True, help='Vision configuration JSON')
        sub.add_argument('--active-tcp', type=float, nargs=6, metavar='V',
                         help='Active TCP offset for read-only use (m, rad); must equal the calibrated TCP')
        sub.add_argument('--pose-store', type=Path, default=Path('data/live/poses.json'))

    observe = commands.add_parser('observe', help='Detect a target; print its base-frame pose')
    common(observe)
    observe.add_argument('--role', choices=('pick', 'place'), default='pick')
    observe.add_argument('--observe-pose', help='Use the recorded joints of this named pose as observe joints')
    observe.add_argument('--save', type=Path, help='Write the observation JSON (input for teach-grasp)')
    observe.add_argument('--execute', action='store_true',
                         help='Connect RTDE control, move to the observe joints and refine (robot moves)')

    teach = commands.add_parser('teach-grasp', help='Store T_marker_tcp from an observation and a taught TCP')
    common(teach, recipe_required=False)
    teach.add_argument('--role', choices=('pick', 'place'), default='pick')
    teach.add_argument('--observation', type=Path, required=True)
    source = teach.add_mutually_exclusive_group(required=True)
    source.add_argument('--grasp-pose', help='Named Robot Bench pose holding the taught TCP')
    source.add_argument('--grasp-tcp', type=float, nargs=6, metavar='V', help='Taught TCP pose (base, m/rad)')
    source.add_argument('--from-robot', action='store_true', help='Read the current TCP pose (RTDE receive)')
    teach.add_argument('--output', type=Path, help='Taught grasp file (default: the role grasp path)')
    teach.add_argument('--force', action='store_true', help='Replace an existing taught grasp')

    pick = commands.add_parser('vision-pick', help='Preview, then execute, a marker-guided pick/place cycle')
    common(pick)
    pick.add_argument('--observe-pose', help='Use the recorded joints of this named pose as pick observe joints')
    pick.add_argument('--observation', action='append', default=[], metavar='ROLE=FILE',
                      help='Preview from a saved observation instead of the camera (pick=..., place=...)')
    pick.add_argument('--observe-motion', action='store_true',
                      help='Preview with observe/refine moves (RTDE control; the robot moves to the observe '
                           'joints and refine views only, never to the targets)')
    pick.add_argument('--preview-out', type=Path, default=Path('data/live/vision_preview.json'))
    pick.add_argument('--write-pick-target', type=Path,
                      help='Also write the pick TCP pose as a six-number JSON array (Robot Bench camera target)')
    pick.add_argument('--execute', action='store_true', help='Connect hardware and run the cycle (robot moves)')
    pick.add_argument('--preview', type=Path, help='Preview file authorizing --execute')
    return parser


class Runtime:
    """Hardware factories; tests replace them with fakes."""

    @staticmethod
    def receive(host):
        from rtde_receive import RTDEReceiveInterface
        return RTDEReceiveInterface(host, 125.0)

    @staticmethod
    def camera(camera_config):
        from .observe import open_camera
        return open_camera(camera_config)

    @staticmethod
    def robot(recipe):
        from ..robot import RTDERobot
        return RTDERobot(recipe)

    @staticmethod
    def connect(recipe):
        from ..session import connect
        return connect(recipe)


def _load(args, runtime):
    from .config import VisionConfig
    from .handeye import check_metrics, load_handeye
    from .planner import file_sha256
    vision_digest = file_sha256(args.vision)
    config = VisionConfig.load(args.vision)
    if file_sha256(args.vision) != vision_digest:
        raise ValueError('Vision configuration changed while it was being loaded; retry')
    calibration = load_handeye(config.path(config.handeye_calibration))
    required_roles = {'pick'} | ({'place'} if config.place is not None else set())
    supplied_roles = ({item.partition('=')[0] for item in args.observation}
                      if args.command == 'vision-pick' else set())
    if (args.command == 'observe' or getattr(args, 'execute', False)
            or getattr(args, 'observe_motion', False) or getattr(args, 'from_robot', False)
            or args.command == 'vision-pick' and not required_roles <= supplied_roles):
        calibration.require_hardware()
    if calibration.camera_serial:
        if config.camera.serial and config.camera.serial != calibration.camera_serial:
            raise ValueError('Configured camera serial differs from the hand-eye calibration')
        config.camera.serial = calibration.camera_serial
    check_metrics(calibration, config.max_handeye_loo_mm, config.max_handeye_reprojection_px,
                  config.max_handeye_translation_sigma_mm)
    recipe_digest = file_sha256(args.config) if getattr(args, 'config', None) else None
    recipe = Recipe.load(args.config) if recipe_digest is not None else None
    inputs = {'vision_config': vision_digest}
    if recipe is not None:
        if file_sha256(args.config) != recipe_digest:
            raise ValueError('Recipe changed while it was being loaded; retry')
        inputs['recipe'] = recipe_digest
    if getattr(args, 'observe_pose', None):
        from ..poses import PoseStore
        saved = PoseStore(args.pose_store).by_name(args.observe_pose)
        if (getattr(args, 'execute', False) or getattr(args, 'observe_motion', False)) and saved.get('demo'):
            raise ValueError('Demo poses cannot be used for hardware execution')
        role = config.role(getattr(args, 'role', 'pick'))
        role.observe_joints = vector(saved['joints_rad'], 6, 'observe joints')
        inputs['observe_pose'] = [args.observe_pose, role.observe_joints]
    return config, calibration, recipe, inputs


def _declared_tcp(args, config):
    return list(args.active_tcp) if args.active_tcp else config.robot.active_tcp_offset


def _print(value):
    print(json.dumps(value, indent=2, allow_nan=False), flush=True)


def _close(resource):
    for name in ('close', 'disconnect'):
        method = getattr(resource, name, None)
        if method is not None:
            method()
            return


def _read_only_observer(args, config, calibration, recipe, runtime, camera_holder):
    from .observe import Observer, check_camera_identity
    from .robot_state import ReceivePoseSource
    receive = runtime.receive(recipe.robot_ip)
    camera_holder.append(receive)
    source = ReceivePoseSource(receive, calibration, _declared_tcp(args, config), config.robot)
    camera = runtime.camera(config.camera)
    camera_holder.append(camera)
    check_camera_identity(camera, calibration)
    return Observer(config, calibration, camera, source, on_step=log.info), source


def _moving_observer(config, calibration, recipe, runtime, held, views=None):
    from .observe import Observer, RobotMover, check_camera_identity
    from .robot_state import ControlPoseSource
    camera = runtime.camera(config.camera)
    held.append(camera)
    check_camera_identity(camera, calibration)
    robot = runtime.robot(recipe)
    held.append(robot)
    source = ControlPoseSource(robot.control, robot.receive)
    return Observer(config, calibration, camera, source, mover=RobotMover(robot), on_step=log.info,
                    views=views), source


def _viewpoint_note(observation, joints):
    moved = max(abs(a - b) for a, b in zip(observation.robot['joints_rad'], joints))
    if moved > 0.01:
        observation.notes.append(f'observed {moved:.3f} rad away from the configured observe joints; '
                                 'execution re-observes from the observe joints')


def cmd_observe(args, runtime):
    config, calibration, recipe, _ = _load(args, runtime)
    role = config.role(args.role)
    held = []
    try:
        if args.execute:
            _print({'mode': 'execute', 'role': args.role, 'observe_joints_rad': role.observe_joints,
                    'refine': role.refine.enabled})
            observer, _ = _moving_observer(config, calibration, recipe, runtime, held)
            observation = observer.look(args.role, allow_motion=True)
        else:
            observer, _ = _read_only_observer(args, config, calibration, recipe, runtime, held)
            observation = observer.look(args.role, allow_motion=False)
            _viewpoint_note(observation, role.observe_joints)
    finally:
        for resource in reversed(held):
            _close(resource)
    _print({'mode': 'execute' if args.execute else 'read-only', 'calibration': calibration.summary(),
            'observation': observation.summary()})
    if args.save:
        atomic_json(args.save, observation.to_json())
        print(f'Observation saved to {args.save}')
    return 0


def cmd_teach(args, runtime):
    from .grasp import teach, write_taught
    from .observe import Observation
    config, calibration, recipe, _ = _load(args, runtime)
    role = config.role(args.role)
    observation = Observation.from_json(json.loads(args.observation.read_text()))
    output = args.output or (config.path(role.grasp) if isinstance(role.grasp, str) else None)
    if output is None:
        raise ValueError('Pass --output or set the role grasp to a file path')
    if args.grasp_pose:
        from ..poses import PoseStore
        saved = PoseStore(args.pose_store).by_name(args.grasp_pose)
        if saved.get('demo'):
            raise ValueError('Demo poses cannot be taught as grasps')
        tcp, source = saved['tcp_pose_m_rad'], f'pose store {args.grasp_pose!r}'
    elif args.grasp_tcp:
        tcp, source = list(args.grasp_tcp), 'command line'
    else:
        if recipe is None:
            raise ValueError('--from-robot needs --config for the robot address')
        from .robot_state import ReceivePoseSource
        receive = runtime.receive(recipe.robot_ip)
        try:
            source = ReceivePoseSource(receive, calibration, _declared_tcp(args, config), config.robot,
                                       expected_tcp=observation.robot.get('tcp_offset'))
            tcp, source = source.snapshot().tcp_pose, 'robot (RTDE receive; TCP checked)'
        finally:
            _close(receive)
    record = teach(observation, tcp, role=args.role, calibration=calibration, limits=role.limits)
    record['taught_tcp_source'] = source
    write_taught(output, record, force=args.force)
    _print({'taught': str(output), 'role': args.role, 'marker_to_tcp_pose_ur': record['marker_to_tcp']['pose_ur'],
            'taught_tcp_in_base': record['taught_tcp_in_base'], 'tcp_offset': record['tcp_offset']})
    print('Keep the same TCP, hand-eye calibration and observe procedure when running vision-pick.')
    return 0


def _preview_observations(args, config, calibration, recipe, job, runtime):
    from .observe import Observation
    supplied = {}
    for item in args.observation:
        role, _, path = item.partition('=')
        if role not in job.roles or not path:
            raise ValueError(f'--observation expects ROLE=FILE with ROLE in {job.roles}')
        supplied[role] = Observation.from_json(json.loads(Path(path).read_text()))
    observations, tcp_offset, held = dict(supplied), None, []
    try:
        missing = [role for role in job.roles if role not in supplied]
        if missing and args.observe_motion:
            observer, source = _moving_observer(config, calibration, recipe, runtime, held, job.views)
            for role in missing:
                observations[role] = observer.look(role, allow_motion=True)
            tcp_offset = source.snapshot().tcp_offset
        elif missing:
            observer, source = _read_only_observer(args, config, calibration, recipe, runtime, held)
            tcp_offset = source.tcp_offset
            for role in missing:
                observations[role] = observer.look(role, allow_motion=False)
                _viewpoint_note(observations[role], config.role(role).observe_joints)
    finally:
        for resource in reversed(held):
            _close(resource)
    if tcp_offset is None:
        tcp_offset = _declared_tcp(args, config) or observations['pick'].robot.get('tcp_offset')
    return observations, tcp_offset


def cmd_vision_pick(args, runtime):
    from .planner import VisionJob
    config, calibration, recipe, inputs = _load(args, runtime)
    job = VisionJob(recipe, config, calibration, inputs=inputs)
    if not args.execute:
        if args.preview:
            raise ValueError('--preview is only used with --execute')
        if args.observe_motion and args.observation:
            raise ValueError('--observe-motion observes every target live; drop --observation')
        observations, tcp_offset = _preview_observations(args, config, calibration, recipe, job, runtime)
        if args.observe_motion:
            recipe.gripper.require_live_ready()
        plan = job.compose(observations, tcp_offset, strict=False)
        record = job.preview_record(plan)
        atomic_json(args.preview_out, record)
        if args.write_pick_target:
            atomic_json(args.write_pick_target, plan['pick_pose'])
        warnings = [f'{role}: observed {"with" if report["observation_refined"] else "without"} the refine view, '
                    'unlike teaching/execution; targets can differ by about the hand-eye error '
                    '(preview with --observe-motion, or teach and run with the same procedure)'
                    for role, report in plan['targets'].items() if report.get('same_procedure_as_teaching') is False]
        _print({'mode': 'preview', 'preview_id': record['id'], 'preview_file': str(args.preview_out),
                'valid_for_s': config.preview.max_age_s, 'gripper': recipe.gripper.kind, 'warnings': warnings,
                'calibration': calibration.summary(), **job.public(plan)})
        print(('Preview complete: observe/refine moves were performed. ' if args.observe_motion else
               'Preview only: no motion. ')
              + 'Controller reachability is checked at execution. Review the poses, then run '
              f'vision-pick --execute --preview {args.preview_out}. '
              'Envelope checks do not validate collision-free paths.')
        return 0
    if args.observation:
        raise ValueError('--execute always observes live; saved observations are for previews')
    if not args.preview:
        raise ValueError('--execute requires --preview FILE from a vision-pick preview')
    preview = json.loads(args.preview.read_text())
    job.check_preview(preview)
    recipe.gripper.require_live_ready()
    from .observe import check_camera_identity
    # Serialize use of this preview before either process can open RTDE control.
    with _preview_lock(args.preview):
        preview = json.loads(args.preview.read_text())
        job.check_preview(preview)
        camera = runtime.camera(config.camera)
        try:
            check_camera_identity(camera, calibration)
            with runtime.connect(recipe) as controller:
                plan = job.execute(preview, controller, camera, on_step=log.info,
                                   consume=lambda used: atomic_json(args.preview, used))
        finally:
            _close(camera)
    _print({'mode': 'execute', 'completed_cycles': 1, **job.public(plan)})
    return 0


@contextmanager
def _preview_lock(path):
    lock = Path(str(path) + '.lock')
    try:
        descriptor = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    except FileExistsError as exc:
        raise ValueError(f'Preview is already in use ({lock}); review the cell before removing a stale lock') from exc
    try:
        os.close(descriptor)
        yield
    finally:
        lock.unlink()


def main(argv=None, runtime=Runtime):
    from ..shutdown import stop_on_termination
    with stop_on_termination():
        return _main(argv, runtime)


def _main(argv=None, runtime=Runtime):
    args = _parser().parse_args(argv)
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    handlers = {'observe': cmd_observe, 'teach-grasp': cmd_teach, 'vision-pick': cmd_vision_pick}
    try:
        return handlers[args.command](args, runtime)
    except KeyboardInterrupt:
        print('Interrupted; no automatic release or recovery move.')
        return 130
    except Exception as exc:
        logging.error('%s; stopped without automatic release or recovery moves', exc)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
