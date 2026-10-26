"""Connection ownership for application integrations."""
from contextlib import ExitStack, contextmanager

from .controller import PickPlaceController


@contextmanager
def connect(recipe):
    """Open live hardware and yield a controller; close without releasing jaws."""
    # Validate recipe geometry and grip calibration before connecting hardware.
    PickPlaceController(recipe).plan()
    recipe.gripper.require_live_ready()
    from .grippers.adapters import connect_gripper
    from .robot import RTDERobot
    with ExitStack() as stack:
        gripper = connect_gripper(recipe.gripper, recipe.robot_ip)
        stack.callback(gripper.close)
        robot = RTDERobot(recipe)
        stack.callback(robot.close)
        yield PickPlaceController(recipe, robot, gripper)
