"""Preview without hardware by default; --execute connects and commands hardware."""
import argparse
from dataclasses import replace
import json
import logging
from pathlib import Path

from .config import Recipe, vector
from .controller import PickPlaceController


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--pick-pose-file', type=Path,
                        help='Camera-derived base-frame TCP pose as a JSON six-number array')
    parser.add_argument('--pose-store', type=Path, default=Path('data/live/poses.json'),
                        help='Named poses recorded with Robot Bench')
    parser.add_argument('--home-pose', help='Use the recorded joint angles of this named pose as home')
    parser.add_argument('--pick-pose', help='Use the recorded TCP pose of this named pose as pick')
    parser.add_argument('--place-pose', help='Use the recorded TCP pose of this named pose as placement reference')
    parser.add_argument('--cycles', type=int, default=1)
    parser.add_argument('--cycle-wait-s', type=float, default=2.0)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true', help='Connect to hardware and run motion')
    mode.add_argument('--dry-run', action='store_true', help='Preview only (default); no hardware connections')
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    try:
        if args.cycles < 1:
            raise ValueError('--cycles must be positive')
        from .config import number
        number(args.cycle_wait_s, 'cycle_wait_s', low=0, high=3600)
        recipe = Recipe.load(args.config)
        if args.home_pose or args.pick_pose or args.place_pose:
            from .poses import PoseStore
            store = PoseStore(args.pose_store)
            overrides = {}
            for name, field, coordinates in ((args.home_pose, 'home_joints', 'joints_rad'),
                                             (args.pick_pose, 'pick_pose', 'tcp_pose_m_rad'),
                                             (args.place_pose, 'place_pose', 'tcp_pose_m_rad')):
                if name:
                    saved = store.by_name(name)
                    if args.execute and saved.get('demo'):
                        raise ValueError('Demo poses cannot be used for hardware execution')
                    overrides[field] = saved[coordinates]
            recipe = replace(recipe, **overrides)
        target = None
        if args.pick_pose_file:
            target = vector(json.loads(args.pick_pose_file.read_text()), 6, 'Camera pick pose')
        plan = PickPlaceController(recipe).plan(target)
        print(json.dumps({'mode': 'execute' if args.execute else 'preview',
                          'cycles': args.cycles, 'gripper': recipe.gripper.kind,
                          'grasp_offset_mm': recipe.grasp_offset_mm,
                          'plan_base_m_rad': plan}, indent=2), flush=True)
        if not args.execute:
            print('Preview only: no hardware connected. Envelope checks do not validate collision-free paths.')
            return 0
        recipe.gripper.require_live_ready()
        from .session import connect
        with connect(recipe) as controller:
            completed = controller.run(
                pick_pose=target, cycles=args.cycles, cycle_wait_s=args.cycle_wait_s)
            print(f'Completed {completed} cycle(s).')
        return 0
    except KeyboardInterrupt:
        print('Interrupted; no automatic release or recovery move.')
        return 130
    except Exception as exc:
        logging.error('%s; sequence stopped without automatic release or recovery moves', exc)
        return 1
