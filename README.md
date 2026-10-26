# ur-pick-place

Standalone Python package for force-guided Universal Robots pick/place. No Qt,
GUI, preset database, or dependency on `Universal_Robot_External_GUI`.

The package executes:

1. Joint move to the configured home waypoint; open the gripper.
2. Linear approach above a camera-derived pick target.
3. Descend along base -Z in force mode until contact or the pick endpoint.
4. Stop force mode, apply any contact backoff and JSON grasp offset, then grip.
5. Verify grip, lift, verify again, joint move through home, verify again.
6. Approach placement, force-descend until contact, stop force mode, verify
   release, and retract from the measured contact pose.

Both OnRobot RG2 and the supplied Arduino servo/pressure-pad firmware are
supported. RG2 is the default, matching the previously tested procedure.

## Install

Use Python 3.12 for hardware operation (validated dependency installation).
Offline planning requires Python 3.10 or newer. Newer interpreters can require
building `ur_rtde` from source, including CMake and a C++ toolchain.

```bash
git clone https://github.com/takiihor/ur-pick-place.git
cd ur-pick-place
python3.12 -m venv .venv
source .venv/bin/activate
pip install -e '.[hardware]'
```

Use `pip install -e .` for offline planning/tests only, `.[robot]` for RG2 plus
RTDE, or `.[hardware]` for RTDE plus Arduino serial support. RG2 XML-RPC uses the
Python standard library. Installation does not connect hardware.

## Configure and preview

```bash
cp examples/rg2.json pick_place.json
# Arduino users: copy examples/arduino.json instead.
# Edit robot_ip, home_joints, poses, workspace, and gripper settings.
ur-pick-place --config pick_place.json --dry-run
```

**Example coordinates, joint angles, widths and forces are illustrative.**
Replace them with your cell's values. `home_joints` contains six joint angles
in radians. Configure TCP and payload on the robot before running; the package
does not infer or change them. Use remote control mode as required by your UR
controller and disconnect competing robot-control applications.

A pose is `[x, y, z, rx, ry, rz]`: translation in **metres**, orientation as a
**rotation vector in radians**, all expressed in the **robot base frame**.
These are TCP target poses, not raw marker poses or roll/pitch/yaw angles.

Preview opens no robot or gripper connections. It checks the configured XYZ
workspace envelope, including possible grasp offsets, lifts and retracts. Live
execution additionally checks home joint limits and candidate Cartesian poses
through the robot controller. These checks do not establish collision-free
paths, tool clearance, or a safe joint path through the scene.

## Run the sequence

After configuring and reviewing the recipe:

```bash
ur-pick-place --config pick_place.json --execute --cycles 1
```

Or use `python -m ur_pick_place` with the same arguments. Omitting `--execute`
previews only. `--cycles N` repeats the same target N times; a failure stops
subsequent cycles. `--cycle-wait-s 2` sets the pause between successful cycles.
`return_home` optionally adds a final joint home move after retracting.

No robot motion is performed by package imports or previews. Live execution
connects an RTDE control session, which uploads its control script. Robot moves
are synchronous: Ctrl+C may be handled only after a move completes. This CLI
is not an emergency stop. Probe endpoints and workspace checks are software
limits; controller/network delays can produce stopping overshoot. Errors do
not trigger automatic release or recovery moves. Connection cleanup attempts
to stop active force mode and the RTDE script without opening the jaws.

## Camera integration

The vision application must transform its marker observation into a desired
TCP grasp pose in the robot base frame. This requires camera/robot calibration,
marker-to-grasp translation and orientation, and the correct configured TCP.
Marker image pixels or a camera-frame position alone are insufficient. Marker
detection, calibration and coordinate transformations belong to the calling
vision application and are not implemented by this package.

Write the resulting six-number array to a JSON file, for example:

```json
[0.4, -0.1, 0.25, 0.0, 3.141592653589793, 0.0]
```

Then preview or execute with that target:

```bash
ur-pick-place --config pick_place.json --pick-pose-file target.json
ur-pick-place --config pick_place.json --pick-pose-file target.json --execute
```

The CLI reads the target file once. For a new observation on each cycle, use
Python and call `run()` separately with each newly calibrated target:

```python
from ur_pick_place import Recipe, connect

recipe = Recipe.load('pick_place.json')
# Replace this with the output of your calibrated camera application.
pick_tcp_pose_in_base = [0.4, -0.1, 0.25, 0.0, 3.141592653589793, 0.0]

with connect(recipe) as controller:  # LIVE robot/gripper connection
    controller.run(pick_pose=pick_tcp_pose_in_base, cycles=1)
```

Every `run()` validates its target and preflights its envelope before moving.
The caller must let exceptions stop the job rather than automatically retrying
or continuing to another target.

## Grasp offset and search settings

Edit these JSON settings:

```json
{
  "pick_backoff_mm": 50,
  "grasp_offset_mm": [0, 0, 10],
  "lift_mm": 100
}
```

`grasp_offset_mm` is an additional XYZ translation in **millimetres along robot
base axes**, applied in position mode after force mode stops. Positive Z goes
up; negative Z goes down. Orientation is unchanged. For a contact above the
pick endpoint, the controller first adds `pick_backoff_mm` along base +Z and
then the grasp offset. At the pick endpoint it applies the grasp offset without
a contact backoff. Set `pick_backoff_mm` to zero if only the explicit offset is
wanted. The loaded lift begins at the resulting grasp pose.

`pick_pose` sets the nominal grasp reference and the lowest pick-search endpoint.
`pick_approach_mm` sets the vertical approach above it. A smaller
`pick_probe.max_descent_mm` can exhaust the search before reaching the endpoint;
that aborts instead of gripping. Earlier contact is accepted only above the
endpoint. Placement always requires force contact; reaching its travel budget
without contact blocks release. `place_pose` sets placement XY/orientation and
approach reference height; release happens at the measured contact pose.

`pick_probe` and `place_probe` each accept:

| Setting | Unit | Meaning |
| --- | --- | --- |
| `contact_force_n` | N | Absolute change in base-frame Fz that counts as contact |
| `command_force_n` | N | Downward commanded force magnitude |
| `speed_mm_s` | mm/s | Compliant Z speed limit |
| `max_descent_mm` | mm | Maximum search travel from the actual approach |
| `timeout_s` | s | Maximum probing time after baseline acquisition |
| `lateral_tolerance_mm` | mm | Allowed lateral drift (default 5) |
| `rotation_tolerance_rad` | rad | Allowed orientation drift (default 0.1) |

Before each probe the backend zeros the F/T sensor, waits, and averages a
baseline. It refreshes `forceMode` at 125 Hz using `initPeriod`/`waitPeriod`,
following the [ur_rtde force-mode pattern](https://sdurobotics.gitlab.io/ur_rtde/pages/examples/force_torque_control/forcemode_example.html).
It monitors force, pose, RTDE timestamp progress, safety mode, drift, travel and
timeout. These software checks are not safety-rated. Force mode must stop
successfully before any grip/release or subsequent position move.

## RG2

`examples/rg2.json` configures the OnRobot XML-RPC service (robot IP, port 41414,
tool index 0 by default). Set `gripper.host` if it differs from the robot IP.
Opening requires target width and no grip feedback; closing requires positive
grip detection. Jaw widths are mm; jaw force is N. The RG2 S1/S2 interlock is
checked during picking by a separate monitor so XML-RPC requests do not block
the RTDE force update loop. Safety faults are reported, never automatically
cleared or treated as pick contact.

## Arduino gripper

The original firmware is in `firmware/arduino_gripper.ino`; its protocol is:

| Command | Reply | Action |
| --- | --- | --- |
| `1\n` | `Tight` | Servo angles 115/90 degrees |
| `2\n` | `Loose` | Servo angles 90/123 degrees |
| `3\n` | `Open` | Servo angles 40/165 degrees |
| `0\n` | `FSR1:... / FSR2:...` | Read both pads |

Serial is 9600 baud, 8N1. Use the appropriate physical connection; for RS485,
use an adapter with automatic transmit-direction control. The firmware controls
its own direction using pin 5; the driver does not set host RTS direction.
Opening a serial connection can reset the Arduino; `startup_wait_s` defaults
to 2 seconds. Replies acknowledge commands, not completed motion.

```bash
arduino-gripper ports
arduino-gripper --port /dev/ttyUSB0 pressure
arduino-gripper --port /dev/ttyUSB0 open
arduino-gripper --port /dev/ttyUSB0 tight
```

Each pad value is the **sum of three raw ADC samples (0..3069)**, not force in N.
Measure empty/open and securely held-object readings with your installed pads,
then configure both `grip_min_raw` and `release_max_raw` in the Arduino recipe.
Its example uses `null` thresholds so hardware sequence execution is blocked
until you supply measured calibration. Larger readings must indicate stronger
pressure for this threshold model; validate your sensor wiring accordingly.

Both pads must exceed their grip thresholds on every configured
`pressure_samples` reading before lifting. Both must fall below their release
thresholds before retracting. Set `settle_s` to allow mechanical motion before
sampling. Set `grip_command` to `tight` or `loose`. The Arduino firmware has no
RG2 safety-switch or calibrated width/force feedback; pressure thresholds are
an application check and cannot prove a physically secure grasp.

## Development

```bash
pip install -e .
python -m unittest discover -s tests -v
```

Tests use fake RTDE/serial hardware and a local XML-RPC server. They cover
sequence order, camera target validation, grasp offsets, early contact,
force refresh/stop failures, travel limits, stale feedback, grip/release
failures, repeats, and both gripper protocols. No hardware motion has been
validated in this standalone package.

The RG2 driver, Arduino driver and firmware were extracted from the user's
`Universal_Robot_External_GUI` project; sequencing and direct RTDE integration
are standalone implementations. The GUI repository is not needed at runtime.
