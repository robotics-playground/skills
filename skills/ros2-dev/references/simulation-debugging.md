# Simulation Debugging: ROS 2 + Gazebo

Debugging a simulated robot is a different discipline from debugging real
hardware or pure software. The simulator is a second source of truth that can
*disagree* with your ROS 2 graph -- and when it does, the disagreement is
invisible unless you deliberately look for it. This reference is about making
that disagreement visible, fast, so you stop chasing downstream symptoms.

## Table of Contents

1. [The core principle: simulator is ground truth](#1-the-core-principle-simulator-is-ground-truth)
2. [The diagnostic ladder for sim issues](#2-the-diagnostic-ladder-for-sim-issues)
3. [Reading the simulator's actual physics pose](#3-reading-the-simulators-actual-physics-pose)
4. [The odometry/physics decoupling failure mode](#4-the-odometryphysics-decoupling-failure-mode)
5. [`use_sim_time` propagation -- the silent killer](#5-use_sim_time-propagation--the-silent-killer)
6. [Robot model: URDF gazebo-extensions vs parallel SDF](#6-robot-model-urdf-gazebo-extensions-vs-parallel-sdf)
7. [Physical-stability failure modes](#7-physical-stability-failure-modes)
8. [Sensor self-occlusion and the gpu_lidar gotcha](#8-sensor-self-occlusion-and-the-gpu_lidar-gotcha)
9. [Viewer-side issues (Foxglove / RViz)](#9-viewer-side-issues-foxglove--rviz)
10. [Clean-state discipline](#10-clean-state-discipline)
11. [Standing diagnostics: the scripts in this skill](#11-standing-diagnostics-the-scripts-in-this-skill)
12. [Symptom -> cause quick table](#12-symptom---cause-quick-table)

---

## 1. The core principle: simulator is ground truth

In a real robot, `/odom` is your best available estimate of where the robot is
-- there is nothing more authoritative to compare against. In simulation, this
is **not true**: the simulator's physics engine knows *exactly* where every
body is. That makes the single most powerful debugging move in sim:

> **Compare what the ROS 2 graph believes against what the simulator knows.**

Almost every confusing sim symptom -- "the laser scan drifts when I turn", "the
map gets corrupted", "the robot won't navigate", "the robot tips over" -- is
downstream of one of a small number of upstream faults. If you debug at the
symptom layer (tweaking sensor configs, QoS, TF buffer depths) you can burn
hours. If you debug by comparing belief vs ground truth, the real fault
surfaces in seconds.

**The mindset shift:** in sim, do not trust `/odom`. Treat it as a *claim* to
be verified against the physics engine, not as fact.

---

## 2. The diagnostic ladder for sim issues

Work top to bottom. Stop at the first rung that reveals a verified cause.
Going further wastes time; stopping early produces a wrong diagnosis.

**Rung 1 -- Is the symptom what you think it is?**
Restate it in concrete, observable terms. "Scan drifts when turning" is vague.
"After commanding a 90 deg rotation, the four cardinal scan ranges did not
rotate by one quadrant" is precise and testable. A square room, for instance,
produces nearly identical scan ranges in every direction -- a robot rotating in
the center of one looks like "the scan isn't moving" even when everything is
correct. Pick an *asymmetric* test condition so the expected change is large
and unambiguous.

**Rung 2 -- Does the robot physically move the way odom claims?**
Run the one-shot motion check (`scripts/sim_motion_check.py`). It commands a
known motion and prints odom-delta vs physics-delta side by side. This one test
eliminates or confirms the entire class of "odometry is lying" bugs. **Do this
before touching sensors, TF, or the viewer.**

**Rung 3 -- Is `use_sim_time` set on every ROS-side process?**
If any node that consumes timestamped data (`robot_state_publisher`, the
bridge, SLAM, EKF, the viewer bridge, your own nodes) runs on wall-clock while
the simulator publishes sim-time, TF lookups fail or use stale data. See
section 5 -- this is extremely common and produces "drift" that is really a
timestamp mismatch.

**Rung 4 -- Is the TF chain complete, fresh, and single-sourced?**
`ros2 run tf2_tools view_frames` for the tree shape. Check publish rates and
that no transform has two publishers fighting over it. A dynamic transform
being published by both a plugin and `robot_state_publisher` causes jitter.

**Rung 5 -- Is the sensor data itself correct in the sensor's own frame?**
Only now look at the sensor. Echo the raw values, reason about the geometry,
check for self-occlusion (section 8). By this rung most "sensor bugs" have
already been found upstream.

**Rung 6 -- Is it a viewer-side display setting?**
If belief matches ground truth, TF is healthy, and sensor data is correct, then
"drift" you see in a viewer is a *rendering* choice -- usually the display
frame (section 9).

The discipline that matters: **do not apply a fix until a rung produces a
verified cause.** "It might be the sensor" is a hypothesis; "the motion check
shows odom diverging from physics by 0.4 m" is a verified cause.

---

## 3. Reading the simulator's actual physics pose

Gazebo's `SceneBroadcaster` system publishes the true world pose of every
dynamic entity on:

```
/world/<world_name>/dynamic_pose/info     (gz.msgs.Pose_V)
```

Bridge it into ROS 2 so your diagnostics can read it:

```yaml
# ros_gz_bridge YAML
- ros_topic_name: "/sim/ground_truth_pose_array"
  gz_topic_name:  "/world/<WORLD_NAME>/dynamic_pose/info"
  ros_type_name:  "geometry_msgs/msg/PoseArray"
  gz_type_name:   "gz.msgs.Pose_V"
  direction:      GZ_TO_ROS
```

**The naming caveat that will trip you up:** the simulator's `Pose_V` message
sets a `name` field on each entity, but the standard `ros_gz_bridge`
conversions (`Pose_V` -> `PoseArray` and `Pose_V` -> `TFMessage`) **drop that
name**. There is no off-the-shelf bridge mapping that preserves per-entity
names. So you address entities by **index**, not name.

When the robot is the only dynamic entity, or the first one spawned, **index 0
is reliably the robot's base link**. Verify once with `ros2 topic echo` (the
other entries will be the robot's wheels/sensors at their joint offsets).

If you have multiple dynamic entities and index-0 is ambiguous, the robust
alternatives are:
- Spawn the robot first so it is deterministically index 0.
- Add a model-scoped pose-publisher plugin to the robot so it emits its own
  `/model/<name>/pose` -- but note this comes from the same source as the drive
  plugin's odometry, so it is *not* an independent ground truth.
- Write a small custom Gazebo system plugin that reads the named `Pose_V` and
  republishes the one entity you care about on its own topic. This is the only
  fully robust option, but it is rarely worth it for a single robot.

---

## 4. The odometry/physics decoupling failure mode

This is the highest-frequency, highest-impact, hardest-to-spot sim bug. Learn
its signature.

**Mechanism.** A drive plugin (diff-drive, mecanum, ackermann) computes
odometry by integrating wheel joint velocities. It is a *dead-reckoning
estimate*, not a measurement of the body. If the chassis is being pushed around
by physics forces the plugin does not know about, odom keeps integrating a
clean number while the real body goes elsewhere.

**What pushes the chassis around independently of the wheels:**
- A belly plate / skid collision shape touching the ground with non-trivial
  friction -- the chassis gets dragged, the wheels slip.
- A free-floating link (a link with no joint, or a joint that did not solve).
- An unstable footprint (two wheels on a single transverse axis with nothing
  fore/aft) so the chassis pitches or tips, and contact geometry changes.
- A wheel jammed against world geometry.
- Wheel friction set so low the wheels spin without traction.

**Signature.** odom and the simulator's ground-truth pose start in agreement
and *diverge progressively while the robot drives*. A few centimeters per
second of position drift, several degrees per second of yaw drift. After ten
seconds they can be half a meter and forty degrees apart.

**Why it produces every downstream symptom.** TF is built on `/odom`. SLAM is
fed that TF and the laser scan; when odom lies, SLAM's scan-matching tries to
reconcile and produces a distorted `map -> odom` correction. The viewer renders
all of it faithfully -- faithfully wrong. So you see "scan drifts", "map
corrupted", "robot won't navigate" -- all one upstream fault.

**How to confirm.** `scripts/sim_motion_check.py` (one-shot) or
`scripts/sim_ground_truth_diagnostic.py` (continuous). Both compare `/odom`
against the bridged ground-truth pose. If they diverge, this is your bug.

**How to fix.** The fix is always in the *physics setup of the robot model*,
never in the sensors or TF:
- Remove ground-contacting collision shapes that are not wheels, or set their
  friction near zero so they slide instead of drag.
- Ensure every link is connected by a joint that actually solves.
- Give the robot a stable footprint -- if it has only two driven wheels on one
  axis, add passive low-friction caster spheres fore and aft so it cannot
  pitch.
- Verify the drive plugin's `wheel_separation` and `wheel_radius` match the
  actual URDF joint geometry. A mismatch here makes odometry wrong even when
  physics is perfectly stable.

---

## 5. `use_sim_time` propagation -- the silent killer

When a simulator runs, it publishes a `/clock` topic and every simulated
message is stamped with *simulation time*, which has no relationship to
wall-clock time. Every ROS 2 node that touches timestamped data must be told to
use sim time, or it will compare sim-stamped messages against its own
wall-clock and conclude the data is billions of seconds stale.

**The trap:** people set `use_sim_time:=true` on the obvious nodes (SLAM, EKF)
and forget the less obvious ones. The ones that are easy to miss:
- `robot_state_publisher` -- builds the TF tree; wrong clock means TF entries
  look ancient.
- The ROS<->sim bridge node itself.
- The visualization bridge (e.g. the Foxglove bridge) -- if it runs on
  wall-clock, it cannot match a sim-stamped scan to a sim-stamped TF, so it
  renders the scan at a stale pose and you see "drift".
- Your own diagnostic / utility nodes.

**The verification, not the assumption.** Do not assume the launch file set it.
Check the *running process*:

```bash
ros2 param get /robot_state_publisher use_sim_time
ros2 param get /foxglove_bridge use_sim_time
ros2 param get /<your_node> use_sim_time
# ... for every node in `ros2 node list`
```

A clean sim has every node reporting `true`. One node reporting `false` is
enough to produce convincing "drift".

**A subtle propagation failure:** some launch files accept a `use_sim_time`
argument but pass it to a downstream node as a *string* parameter, where the
node expects a *bool*. The node then either crashes on a type error or silently
ignores it. If a launch-arg approach is not taking effect, invoke the node
directly with `--ros-args -p use_sim_time:=true` (proper bool) instead of going
through the launch file's argument plumbing.

---

## 6. Robot model: URDF gazebo-extensions vs parallel SDF

There are two ways to get a robot into Gazebo, and choosing wrong causes a
whole category of frame-mismatch bugs.

**Option A -- one URDF with `<gazebo>` extension blocks (recommended).**
Author the robot once in URDF/Xacro. Add `<gazebo>` blocks for the
Gazebo-specific bits: sensor `<sensor>` definitions, the drive plugin,
joint-state and pose publishers, per-link materials and friction. Spawn it into
the simulator from the `/robot_description` topic (e.g. via `ros_gz_sim
create -topic /robot_description` or `-file <processed_urdf>`).

The single URDF feeds **both** `robot_state_publisher` (which builds the ROS 2
TF tree) **and** the simulator (which builds the physics model). Because there
is one source of truth, the TF frames and the physics frames are identical by
construction. This is the canonical pattern in current ROS 2 + Gazebo
reference projects.

**Option B -- a separate SDF model maintained alongside the URDF.**
You write the robot twice: a URDF for `robot_state_publisher` and a
hand-authored SDF model embedded in or included by the world file. This is
fragile: the two descriptions drift apart. The classic failure is a link whose
pose or origin is defined slightly differently in the two files -- now the TF
tree says the sensor is in one place and the physics engine has it in another,
and every sensor reading is transformed through a wrong offset. The mismatch is
silent; nothing errors.

**If you inherit a parallel-SDF setup and see frame weirdness**, the highest-
value move is to unify on Option A: convert the SDF robot's links/joints/
plugins into `<gazebo>` blocks inside the existing URDF and spawn from
`/robot_description`. It eliminates an entire bug class rather than patching
instances of it.

---

## 7. Physical-stability failure modes

A simulated robot that is physically unstable will produce sensor and odometry
data that looks "drifty" or "jumpy" -- because the body really is moving in
ways the navigation stack does not model. Common causes and fixes:

**Two driven wheels on a single transverse axis, nothing fore/aft.**
The robot balances on a line and pitches forward or backward. The fix is to
give it fore/aft support: passive low-friction caster spheres at the front and
rear of the chassis. Set their friction near zero (`mu1`/`mu2` ~ 0.01) so they
slide freely and do not fight the drive wheels. Do *not* use a high-friction
belly plate for this -- that reintroduces the decoupling fault from section 4.

**Wheels that "roll" around the wrong axis.**
A cylinder's default axis is its local Z. A wheel that should roll forward must
have its cylinder axis along the robot's Y (left-right). If the rotation axis
is wrong, spinning the joint makes the wheel pirouette in place instead of
rolling, and the robot skids unpredictably. Verify the wheel link's visual/
collision `<origin>` rotation and the joint `<axis>` together produce
roll-about-Y.

**Inertia tensors that are zero, tiny, or physically impossible.**
A link with near-zero inertia gets flung by the constraint solver. Use the
correct box/cylinder inertia formulas for the link's mass and dimensions.
A 5 kg chassis with `ixx=ixy=...=1e-9` will not behave.

**Collision shapes coincident with the sensor ray origin.**
See section 8 -- this is a self-occlusion bug, not a stability bug, but it
shares the "sensor body collision shape" root.

**Verification.** After any model change, read the simulator's ground-truth
pose immediately after spawn (before driving). It should be at the spawn pose
with an identity-ish orientation. A non-trivial roll or pitch a second after
spawn means the robot is settling into an unstable equilibrium -- fix the
footprint before doing anything else.

---

## 8. Sensor self-occlusion and the gpu_lidar gotcha

Simulated ray sensors (`gpu_lidar`, `gpu_ray`, depth cameras) cast rays from
the sensor's origin. If a collision shape sits at or very near that origin --
including the *sensor's own link collision shape* -- the rays can register a
hit on it immediately, clipping the scan to a tiny range or carving a dead
wedge out of the field of view.

**Rules that avoid it:**
- A sensor link should generally have **no `<collision>`** -- only `<visual>`
  and `<inertial>`. The sensor body does not need to physically collide with
  anything; a coincident collision shape only occludes the sensor's own rays.
- If the sensor link must have a collision shape for some reason, keep it small
  and offset from the ray origin.
- Mount the sensor high enough that the robot's own chassis and other
  structures are below the scan plane. A LiDAR whose scan plane intersects a
  turret, mast, or tall payload will see those as obstacles -- realistic, but
  often not what you want for SLAM.

**Distinguishing self-occlusion from a real obstacle:** drive the robot and
watch the suspect beam. If the range is *constant* regardless of where the
robot moves, the "obstacle" is attached to the robot -- it is self-occlusion.
If the range changes with robot position, it is a real world object.

**A trap when investigating this:** if the robot is *stuck* (section 4 / 7),
its physical position never changes, so *every* beam looks constant and
*everything* looks like self-occlusion. Always confirm the robot is actually
moving in physics (rung 2) before concluding "self-occlusion".

---

## 9. Viewer-side issues (Foxglove / RViz)

If the data is correct (belief matches ground truth, TF healthy, sensor values
correct) but a viewer *shows* something wrong, the bug is in the viewer's
display configuration.

**Display frame.** A 3D panel renders everything relative to a chosen "display
frame". If it is set to a robot-attached frame (`base_link`, the sensor frame),
the robot appears fixed and the *world* appears to move and rotate around it --
which a user naturally describes as "the map drifts" or "the walls rotate with
the robot". For a world-fixed view, set the display frame to `map` or `odom`.
This is the single most common "it's drifting" report that is not actually a
data bug.

**TF buffer / QoS history depth.** Viewers maintain their own TF buffer from
the `/tf` topic. If the viewer's subscription QoS history depth is too shallow
(some default to a depth of ~10) and `/tf` publishes fast (often 50-100 Hz),
the buffer holds only a fraction of a second of history. A sensor message
stamped slightly in the past then has no matching TF entry, and the viewer
falls back to the latest transform -- so the scan renders at a stale pose and
appears to lag or "stick" during motion. The fix is to raise the viewer
bridge's max QoS history depth (e.g. to several hundred) so the TF buffer holds
a few seconds. Watch the bridge's logs for messages about "limiting history
depth" -- that warning is the tell.

**Invalid-value rendering.** Ray sensors legitimately return `+inf` for beams
that hit nothing within max range. Viewers often render these with an
alternating error pattern. That is a *display* choice, not bad data -- set the
panel's invalid-value handling to hide or clamp.

---

## 10. Clean-state discipline

Simulations accumulate degenerate state. A robot that drove through a wall, an
odometry integrator that ran for an hour, a SLAM map built on bad TF -- these
persist across "restarts" that are not actually clean. Debugging on top of
corrupted state produces meaningless measurements and wasted hours.

**Practices:**
- When a measurement looks impossible (odometry reading billions of meters, a
  robot pose far outside the world), **suspect stale state first**, not a new
  bug. Restart fully and re-measure.
- A "restart" that reuses a long-lived process may not reset integrator state
  or rebuild the model. For a genuinely clean slate, tear the sim process down
  and bring it back up -- not just a soft restart.
- Snapshot known-good state explicitly. Before a risky change, note the working
  configuration so you can return to it deterministically instead of trying to
  reconstruct it.
- Reset the robot's pose between tests rather than letting it accumulate drift
  and end up jammed against world geometry. If the simulator's pose-reset
  service is available, use it; otherwise restart.
- One change at a time. Changing the model, the bridge config, and the launch
  file together and then testing tells you nothing about which change did what.

---

## 11. Standing diagnostics: the scripts in this skill

This skill bundles two ready-to-use diagnostic scripts in `scripts/`. They are
generic and parameter-driven -- copy them into any ROS 2 package, expose them
as `console_scripts` entry points, or run them directly.

**`scripts/sim_ground_truth_diagnostic.py` -- continuous monitor.**
A long-lived node that subscribes to `/odom` and the bridged ground-truth pose,
and publishes a one-line comparison on `/diagnostic/odom_vs_physics` every
second. Logs a one-shot WARN the instant odom and physics diverge past
threshold, and a one-shot INFO when they recover. Run it from the sim
container's entrypoint so the comparison is *always available* -- then any
future "the robot is acting weird" question is answered by
`ros2 topic echo /diagnostic/odom_vs_physics` instead of by an hour of guessing.

**`scripts/sim_motion_check.py` -- one-shot sanity test.**
Run it interactively when something seems wrong. It commands a known motion
(drive forward, then rotate), records odom-delta vs physics-delta, and prints a
verdict: MOTION SOUND / MOTION DECOUPLED / ROBOT DID NOT MOVE. This is the
single most useful first move when debugging any sim motion, mapping, or sensor
issue -- it tells you in one run whether the bug is upstream (physics) or
downstream (sensors / TF / viewer).

Both scripts require the ground-truth pose to be bridged (section 3) and
`use_sim_time:=true`. Each script's docstring has the exact bridge line and
usage.

**Why bundle these rather than re-derive them each time:** during a hard sim
debug it is tempting to write throwaway comparison scripts inline, over and
over. That is slow and the throwaway versions are inconsistent. A standing
diagnostic that is always running turns the highest-leverage check into a
zero-effort glance.

---

## 12. Symptom -> cause quick table

| Symptom | Most likely cause | Where to look |
|---|---|---|
| Laser scan "drifts" / "rotates with the robot" | Viewer display frame set to a robot-attached frame | Section 9 -- set display frame to `map`/`odom` |
| Scan, map, AND robot all drift together when turning | odometry/physics decoupling | Section 4 -- run `sim_motion_check.py` |
| Map gets progressively corrupted / distorted | SLAM fed a lying TF from bad odometry | Section 4 -- fix odometry first, SLAM is a victim |
| Scan appears "stuck" / lags during motion | Viewer TF buffer too shallow for fast `/tf` | Section 9 -- raise viewer bridge QoS history depth |
| Everything stamped "billions of seconds" stale | A node missing `use_sim_time` | Section 5 -- check `use_sim_time` on every node |
| Robot tips / pitches / rolls shortly after spawn | Unstable footprint | Section 7 -- add fore/aft caster support |
| Robot commanded to move but barely moves in physics | Robot stuck (world collision, jammed joint, no friction) | Section 7 / rung 2 |
| odom says the robot moved but it visibly did not | odometry integrating phantom motion; robot stuck | Section 4 -- `sim_motion_check.py` says "ROBOT DID NOT MOVE" |
| A scan beam reads a constant short range everywhere | Sensor self-occlusion (own collision shape / chassis in plane) | Section 8 -- but first confirm robot actually moves |
| Sensor frame and physics disagree by a fixed offset | Parallel-SDF / URDF model drift | Section 6 -- unify on URDF + `<gazebo>` blocks |
| Wheel spins but robot skids / pirouettes | Wheel rotation axis wrong | Section 7 -- check wheel `<axis>` is roll-about-Y |
| Odometry scale is consistently off (over/under-shoots) | Drive plugin `wheel_radius`/`wheel_separation` ne URDF | Section 4 -- match plugin params to joint geometry |
| Measurement looks physically impossible | Stale / degenerate sim state | Section 10 -- full restart, re-measure |

---

## The one-sentence summary

In simulation you have something you never have on real hardware -- the
engine's ground truth -- so the fastest path through almost any confusing sim
bug is to compare what the ROS 2 graph believes against what the simulator
knows, and fix the first place they disagree.
