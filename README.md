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

## Start the backend and teaching UI

From a clone of this repository:

```bash
chmod +x start.sh  # already executable in Git
./start.sh
```

`start.sh` finds Python 3.12 (or 3.10–3.13), creates `.venv`, installs the
package and hardware dependencies, starts the local backend, and opens Robot
Bench at **http://127.0.0.1:8765**. Subsequent starts reuse the environment;
dependencies are refreshed when package requirements change. Set
`PYTHON_BIN=/path/to/python3.12` before the first start to select an interpreter.
No system packages are installed. Creating a virtual environment requires
Python's `venv` support on your system.

The backend starts disconnected: enter your robot IP and click **Connect
RTDE**. Configure TCP/payload and remote control on the robot as appropriate.
Only one application should own RTDE control. Disconnect the original GUI or
other robot controllers before connecting Robot Bench.

```bash
./start.sh --demo                     # try the UI without robot hardware
./start.sh --config pick_place.json   # initialize IP, workspace and speed limits
./start.sh --no-browser --port 8765    # backend only; open the URL yourself
./start.sh setup                      # install dependencies without starting
./start.sh test                       # offline tests
```

The same backend can be started with `ur-robot-ui` after package installation.
It serves only on loopback. `--demo` never imports/connects RTDE and keeps its
settings/poses separate from live recordings.

### Two-page workflow

Use the navigation at the top of Robot Bench:

1. **Record waypoints** (`/waypoints`) shows live TCP/joint feedback, positioning
   controls, pose recording, saved names and renaming. Position the robot here
   and save each waypoint.
2. **Joint & force tests** (`/testing`) lets you select a saved waypoint and
   press **Test joint move**, or choose Home/Pick/Place plus **OnRobot RG2** or
   **Arduino gripper** and preview/run a complete force-guided cycle. It also
   shows live TCP/orientation/joint feedback while testing.

The robot connection, saved waypoints, gripper settings and sequence status are
shared between the pages. Navigation keeps unfinished form values and an
active test intact. **Stop motion** remains available in the header on both
pages. The root URL opens the waypoint page; either page can be opened directly
or bookmarked. Switching pages does not create another RTDE connection.

### Test the full sequence with buttons

On **Joint & force tests**, the **Pick & place test** panel runs the complete cycle using the UI's existing
RTDE connection. Keep Robot Bench connected; no separate terminal or second
robot-control session is needed.

1. Record your home, pick and placement poses in the pose library.
2. Select them in **Home**, **Pick** and **Place**. Home uses recorded joints;
   pick and place use recorded TCP coordinates.
3. Choose **OnRobot RG2** or **Arduino gripper** from the **Gripper** dropdown.
   Both drivers remain available and can be switched between tests.
4. Set the gripper's values. RG2 uses open/grip widths and gripping force;
   Arduino uses its serial port, tight/loose command and both pads' calibrated
   grip/release thresholds.
5. Expand **Force, approach & grasp offset** to adjust the approach heights,
   contact/command forces, descent limits/speeds, loaded lift, retract distance,
   earlier-contact backoff and base-frame XYZ grasp offset.
6. Click **Preview cycle**, review the selected poses, gripper and parameters,
   then click **Run one cycle**. Changing settings requires another preview.

The six-stage progress strip shows home/open, pick approach, force search,
offset/grip, verify/lift/home, and force-place/retract. Each run consumes its
preview and performs exactly one cycle. Manual movement and pose recording
are blocked during a cycle. The persistent **Stop motion** button cancels the
sequence, stops active force mode or an arm move, and blocks subsequent steps.
It does not open the gripper or command a recovery move. A lost UI heartbeat
also cancels the sequence. The robot connection remains available after a
normal cancellation; inspect the robot and gripper before testing again.
Gripper communication is bounded by its configured timeout, so cancellation
can wait for an outstanding jaw/status reply. This is not an emergency stop.

**Camera target (optional)** accepts a pasted or loaded JSON six-number TCP
pose in base-frame metres/radians. A supplied camera target overrides the saved
pick pose. Detector/calibration processing still belongs to your camera app.
The test choices are saved in `data/live/sequence.json` (or `data/demo/sequence.json`).
Starting with `--config pick_place.json` initializes the recipe's other defaults.

To try the complete button flow without hardware:

```bash
./start.sh --demo
```

Click **Connect RTDE**, **Load demo poses**, **Preview cycle**, then **Run one
cycle**. Demo mode simulates both the robot and selected gripper and never
contacts robot or gripper hardware. The force search is accelerated in the
demo. Live sequence motion still requires validation on your actual cell.

### Record and recall poses

The UI shows live base-frame TCP position, rotation-vector orientation and all
six joint angles. To teach a position:

1. Connect, then position the TCP using the hold-to-move buttons or absolute target.
2. Wait for motion to finish, enter a name such as `home_a`, and click
   **Record current pose**.
3. To replace a waypoint, move to the new position and click **Update from current**
   on its saved card, or enter the same name and click **Update saved waypoint**.
   This updates both TCP coordinates and joint angles, keeping its ID and name.
   Updates require fresh feedback and a stationary robot. Preview any sequence again
   after updating its waypoints. Use **Rename** to change a name; names must be unique.
4. **Load target** fills the absolute target fields without moving anything.
   **Move TCP** recalls the saved TCP pose with `moveL`; **Move joints** recalls
   its recorded joint angles with `moveJ`.

Records are saved immediately and atomically to `data/live/poses.json`, with
TCP coordinates in metres/radians, joint angles in radians, a timestamp and the
robot address. **Export JSON** downloads the same portable store. Files persist
across restarts. Demo records live in `data/demo/poses.json` and cannot be used
for live motion. Use `--data-dir PATH` to choose a different storage directory.

### TCP movement controls

- Hold a position button to jog continuously along robot-base X, Y or Z.
  Release to stop. The movement speed slider is in mm/s, up to the configured TCP speed limit
  (maximum 100 mm/s). Jogging starts at 20 mm/s, or the configured limit if lower.
- Orientation buttons compose a physical rotation about robot-base X, Y or Z,
  at the selected deg/s; they do not add Euler angles to the rotation-vector components.
- Hold with a mouse, touch, or Space/Enter on a focused button. Orientation
  speed slider starts at 5 deg/s and is adjustable up to 10 deg/s. Release, lost focus, navigation, or a
  missing held-button heartbeat stops jogging. The backend checks workspace
  and robot safety limits ahead of motion; other moves are blocked while jogging.
  A continuous hold is limited to 120 seconds; release and press again to resume.
- The absolute target accepts XYZ in mm and Rx/Ry/Rz **rotation-vector
  components in degrees**, not roll/pitch/yaw. **Use current** copies live
  feedback. **Move to TCP target** sends the explicit linear movement.
- **Stop motion** interrupts an asynchronous movement through RTDE. Closing
  the backend stops its motion and control script; losing fresh robot feedback
  or the UI heartbeat stops motion and disconnects. Keep the tab active during
  movement. This is a software stop, not an emergency-stop function.

Expand **Connection & motion limits** while disconnected to set TCP/joint
speeds and XYZ workspace limits. Defaults are 20 mm/s TCP and 0.1 rad/s joint
speed. Limits must cover the actual current TCP pose and every requested
endpoint. The robot's own reachability/safety checks also run before movement;
whole-link collisions and intermediate joint trajectories are not validated.

### Use taught poses in the pick/place CLI

Disconnect Robot Bench before running a pick/place job so the CLI can own the
RTDE control connection. Recorded names can replace the recipe's coordinates:

```bash
./start.sh pick-place --config pick_place.json \
  --home-pose home_a --pick-pose pick --place-pose drop --dry-run

# Add --execute instead of --dry-run after reviewing the configured job.
```

`--home-pose` uses saved joint angles; `--pick-pose` and `--place-pose` use saved
TCP coordinates. The default `--pose-store` is `data/live/poses.json`.
`--pick-pose-file target.json` can still override the pick TCP pose with a new
camera-derived target. Preview can inspect demo records via `--pose-store`,
but live execution rejects records marked as demo.

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


## Wrist-camera vision targeting

`ur-vision` adds `observe`, `teach-grasp` and `vision-pick` to the existing force-guided
cycle. It supports a standalone ArUco marker or a ChArUco board on the object, plus an
optional marker-defined placement fixture. This path is CLI-only; the UI can accept
its exported six-number pick target through **Camera target**.

**Status:** validated with rendered images and fake RTDE/grippers. It has not run on
a physical UR10e, RealSense or URSim. Complete the hand-eye calibration's first
`collect --dry-run`, then collect slowly with a hand on the e-stop. Validate the
pick/place views and paths at reduced speed with the same attendance.

### Install and configure

Use Python 3.11-3.13 (3.12 recommended) and keep `charuco-generator`,
`charuco-realsense` and `ur10e-handeye` beside this repository. The launcher installs
vision dependencies and the first two sibling packages editable with `--no-deps`
to avoid installing conflicting OpenCV distributions. Manual equivalent:

```sh
.venv/bin/python -m pip install -e '.[hardware,vision]'
.venv/bin/python -m pip install --no-deps -e ../charuco-generator -e ../charuco-realsense
cp examples/rg2.json pick_place.json
cp examples/vision-aruco.json vision.json
# Or copy examples/vision-charuco.json for the generated front board.
```

Edit both copies for your cell. The example joint coordinates are placeholders,
not taught viewpoints. Record an observe waypoint with Robot Bench, disconnect its
RTDE control session, then use `--observe-pose NAME` or copy its joints into
`pick.observe_joints`. Use the actual gripper TCP and payload. Set
`robot.active_tcp_offset` to the TCP offset recorded in the hand-eye result for
read-only observation, or pass its six values with `--active-tcp`.

Paths inside `vision.json` are relative to that file. `handeye_calibration` reads
schema 1 from `ur10e-handeye`; unknown versions, inconsistent transforms and `poor`
verdicts are refused. The example requires camera translation 1-sigma <= 0.5 mm on
every axis and reprojection RMSE <= 1 px. Set these to your validated tolerances.
`max_handeye_loo_mm` is available as an additional check, but leave-one-out is
optimistic for camera mount error; quote `camera_translation_sigma_mm` instead.

ArUco marker sizes refer to the black outer square, excluding the white quiet zone.
An `aruco_manifest` can supply and check the size/dictionary. ChArUco targets use
`board_manifest` and `board`, including board geometry and ID checks. Keep a white
quiet zone around each target and enough image resolution (default >= 5 px/module).

### Observe and teach the relationship

Start with a read-only view from the current robot pose:

```sh
./start.sh vision observe --config pick_place.json --vision vision.json \
  --save data/live/pick-observation.json
```

This opens RTDE receive and the camera. It checks the declared TCP against the
calibration and nominal UR10e kinematics, rejects motion during capture, averages
accepted corners and reports pose, resolution, jitter and fit quality. Control
sessions recover the flange with the controller's calibrated FK and a zero TCP,
so changing the active TCP does not change the camera-in-flange transform.

To observe from the configured joints, add `--execute` (robot moves). A role's
optional `refine.enabled` adds a closer frontal view, bounded by translation and
rotation limits. Use the same procedure for teaching and execution. Refined grasps
store the marker-relative camera viewpoint so execution can reproduce it.

Keep the object fixed after saving the observation. Jog/hand-guide to the desired
pick endpoint, save a live waypoint named `Pick grasp`, then disconnect Robot Bench:

```sh
./start.sh vision teach-grasp --vision vision.json \
  --observation data/live/pick-observation.json --grasp-pose 'Pick grasp'
```

Alternatively use `--grasp-tcp X Y Z RX RY RZ` (base metres/radians), or
`--from-robot --config pick_place.json` for a receive-only TCP read. Keep the same
TCP throughout observation and teaching. Teaching writes `grasp_pick.json` and
refuses overwrite unless `--force` is explicit. The marker-to-TCP relation is
`inv(T_base_marker_taught) @ T_base_tcp_taught`.

A `place` object in `vision.json` has the same structure as `pick`; give it another
marker/board, observe joints and `grasp_place.json`, and repeat observe/teach with
`--role place`. If omitted, placement uses the recipe's fixed `place_pose`.

### Preview and run one cycle

```sh
# Current pose, read-only; saved observations can preview entirely offline:
./start.sh vision vision-pick --config pick_place.json --vision vision.json
# Offline alternative: add --observation pick=data/live/pick-observation.json
# Add --observation place=FILE as well when a place role is configured.

# Explicit motion to reproduce the observe/refine procedure, without a pick cycle:
./start.sh vision vision-pick --config pick_place.json --vision vision.json \
  --observe-motion --preview-out data/live/vision_preview.json

# Review the preview, then execute once (robot and gripper move):
./start.sh vision vision-pick --config pick_place.json --vision vision.json \
  --execute --preview data/live/vision_preview.json
```

A fresh observation must match the preview (default 3 mm / 1 degree, <= 15 minutes
old). Changed config, recipe, calibration or grasp files invalidate it; each
preview authorizes one attempt. Read-only previews report when their view procedure
differs from teaching. Execution requires the taught procedure, matching TCP,
limited displacement/tilt, workspace/envelope checks and controller safety/IK
checks. These checks do not establish collision-free paths.

`motion_model: planar` follows XY/yaw while retaining taught height/tilt;
`full` follows the full measured pose. `offset_mm` is applied in target coordinates
before the taught relation. The recipe's `grasp_offset_mm` remains a separate
base-frame offset after force contact. The vision pick pose is the force-search
endpoint; contact backoff, grasp offset, grip verification and placement still use
the existing controller. Ctrl+C or errors close the control session without an
automatic jaw release or recovery move.

`--write-pick-target target.json` exports a six-number base-frame TCP pose for the
UI's Camera target input. This export does not move hardware.

### Offline validation

```sh
.venv/bin/python -m unittest discover -s tests -v
```

Install the vision extra and sibling packages to include all camera tests. Rendered
ArUco/ChArUco images exercise detection, multi-frame gates, calibrated flange/TCP
conventions, taught relationships, shifted objects, optional refine views, preview
consumption, calibration rejection and simulated force-guided pick/place.
