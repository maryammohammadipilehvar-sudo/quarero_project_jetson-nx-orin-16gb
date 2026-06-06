# Learning notes — autonomy fixes, 2026-05-17

This document walks through every problem we hit during the autonomous-mode
session and how we fixed each one. It is written for someone who is learning
the codebase. The goal is that you can read this later and explain each fix
in your own words.

End-to-end result of the session: the robot drove through a 3-waypoint
mission autonomously, reaching each waypoint to within **1–11 cm**.

> **How to read this:** every issue has the same four sections — *Symptom*
> (what you saw), *Why* (the root cause in plain language), *Fix* (what we
> changed and where), *Lesson* (what to remember next time).

---

## Quick orientation: the moving parts you need to know

- **`wp_follower`** (`tactical_wp_follower_node.py`) — the brain. Decides
  what to do in each state (IDLE, NAVIGATING, UNDOCKING, etc.) and sends
  drive commands to the motors.
- **`roboclaw_wrapper`** (`roboclaw_wrapper_node.py`) — the hands. Takes
  `/cmd_drive` messages and writes them to the Roboclaw motor board over USB.
- **`web_app`** container — runs the FastAPI web UI you click in. It has its
  own ROS node that talks to `wp_follower` via ROS services.
- **`/fixposition/...`** topics — published by the Fixposition GNSS unit.
  Position, heading, RTK status all come from here.
- **state machine** inside `wp_follower`: MANUAL → IDLE → DOCKED → CHARGING
  → UNDOCKING → NAVIGATING → RETURNING_TO_HOME. The robot only autonomously
  drives in NAVIGATING.

---

## Issue 1 — "GNSS2 is not green"

### Symptom
The web UI's GNSS2 badge was red. The operator worried autonomy would not
work because both antennas need RTK Fixed.

### Why
It was a transient GNSS2 glitch. By the time we looked at live data,
**both** `gnss1_status` and `gnss2_status` were already `8` (RTK Fixed).
The badge had recovered.

### Fix
None needed. We confirmed live data via:
```bash
ros2 topic echo /fixposition/fusion --once | grep -E "gnss[12]_status|init_status"
```

### Lesson
The web UI badge is tied directly to `gnss1_status` / `gnss2_status` from
the message (`static/js/index.js:530-557`). If it goes red briefly, check
the live topic before assuming a hardware fault. RTK heading needs **both**
antennas to have open sky for a few seconds.

---

## Issue 2 — Misleading variable name in the RTK gate

### Symptom
I (Claude) initially thought the fusion gate had a bug — it seemed to be
checking the wrong field.

### Why
In `rtk_status_monitor.py:213` the code checks
`self._fusion_state.fusion_status == 2`. The Fixposition message
`/fixposition/fusion` has **two** fields that look similar:
- `fpa_odomstatus.init_status` — values 0/1/2 (`2 = Globally initialised`)
- `fpa_odomenu.fusion_status` — values 0..4 (the fusion mode)

The code reads `init_status` but stores it in a dict key literally named
`'fusion_status'` (see `wp_follower:1748`). So when you read the monitor
class, "fusion_status" actually means "init_status".

### Fix
None — the wiring is correct, only the name is misleading. I added a
memory note so future sessions don't get tripped up.

### Lesson
When you see `ros2 topic echo` output with two same-named fields,
**don't trust the name**. Trace where the value goes through the code.
Field renames at module boundaries are common — and a real trap.

---

## Issue 3 — Mission did not run (Run 1)

### Symptom
Operator toggled "Autonomous Operation" on. The robot did nothing for
112 seconds. GPS span over the whole run: **2 cm**.

### Why
`active_route` in `/robot/state` was empty (`name: '', waypoints: []`)
the whole time. Without an active route, the state machine has nothing
to dispatch. Toggling autonomy alone does not load a route — you have
to select one *and* dispatch it.

The "select route" dropdown on the main screen only puts the route on
the **map for preview** (`index.js:705` is the only place it reads). It
never calls a service to load the route into the wp_follower.

### Fix (two parts — Issues 4 and 5 below)
- A cleaner route-delete flow so the operator can manage routes.
- A new "Run Now" button + endpoint that actually dispatches a route.

### Lesson
The state-machine flag `autonomous_enabled` (gate) and the *active route*
(what to do) are two independent things. Both must be set before the
robot drives. UI buttons that "select" routes do not always dispatch
them — read the JS to confirm what the click actually triggers.

---

## Issue 4 — Deleted route stayed on disk

### Symptom
Operator pressed "Route löschen" on `sun1`. The route still appeared in
the picker, and `~/gits/routen/routes/sun1.yaml` was still on disk.

### Why
`route_service.delete_route` had this guard:
```python
if schedule_uses_this_route:
    return False, "Route wird in Schedule genutzt..."
```
It refused to unlink the file if **any** schedule referenced the route.
The web UI showed an alert with that error, but the operator missed it.

### Fix — cascade delete
`route_service.py:delete_route` now:
1. Opens `schedules.yaml`, removes the route name from every schedule's
   `routes` list.
2. If a schedule's `routes` list becomes empty, sets `active: false`
   (keeps the schedule for re-use later — does not delete it).
3. Writes `schedules.yaml` back.
4. **Then** unlinks the route YAML.
5. Returns a summary so the UI can tell the operator what happened.

API: `DELETE /api/routes/delete/{name}` returns
`{status, cleaned_schedules, deactivated_schedules}`.
UI: `index.js` shows those in the success alert.

### Lesson
When you build a "safety guard" that refuses an action, **make the error
unmissable** or auto-resolve the conflict. A flashed alert and a
mysteriously-undeleted file is the worst of both worlds.

---

## Issue 5 — `active_route` never updated from the UI

### Symptom
After "fixing" the delete (Issue 4), the operator selected a route from
the main-screen dropdown and toggled autonomy. Nothing happened. Same
empty `active_route`.

### Why
There was no UI button → backend endpoint → ROS service path to start
a route. The only callers of the wp_follower's
`_start_route_from_schedule_response()` (the function that loads
waypoints into `active_route`) are inside the **scheduler-response
handler**. So routes only got loaded when the scheduler fired on its
own — which would only happen if you waited inside the schedule's time
window for the next cron tick.

### Fix — Run Now
1. New endpoint `POST /api/routes/start_now/{name}` in
   `api/routes/routes.py`. It loads the route YAML and calls the
   already-existing `call_waypoint_service` helper.
2. That helper publishes a request to the existing ROS service
   `/control/waypoints` advertised by `wp_follower`.
3. The wp_follower's handler `_waypoint_service` puts the waypoints
   into `_current_route`, which is what `/robot/state.active_route` is
   built from.
4. New button **"Route jetzt starten"** in `static/index.html` →
   `startSelectedRouteNow()` in `index.js`.
5. To wire the ROS node into the routes router, I added
   `init_routes_router(node)` and call it from `main.py:255`.

### Lesson
"Selecting" in the UI is usually a display action. "Starting" is a
service call. Always trace the JS handler to the network call (`fetch`)
to confirm what really fires.

---

## Issue 6 — `/control/waypoints` crashed the wp_follower every call

### Symptom
First time we hit Run Now, the API timed out after 10 s. The wp_follower
process **died**. Subsequent runs killed it again.

### Why
Two pre-existing bugs in `_waypoint_service` (lines 1862-1945):
1. Line 1874: `geopath.route_name = route_name` — **`GeoPath.msg` has no
   `route_name` field**. Python raised `AttributeError` and the service
   callback never sent a response.
2. Line 1899: `self._waypoint_follower.set_waypoints(waypoint_list,
   WaypointMode.PING_PONG)` — `set_waypoints` only takes one positional
   arg. Mode goes through `set_mode()`. Wrong API call, immediate crash.

Both bugs had been there since the file was written. Nobody noticed
because nothing was actually calling `/control/waypoints` until we
added Run Now.

### Fix
- Removed the dead `geopath.route_name = ...` line.
- Replaced the broken manual unrolling of "set mode then set waypoints"
  with a single call to `_start_navigation_directly(geopath, route_name)`
  — the **same function** the scheduler path uses. That function does
  the map-origin setup, GPS→map transform, set_mode, set_waypoints, and
  a sanity check, all correctly.

### Lesson
"Dead code" that crashes only because nobody calls it is a **time bomb**.
When you write a service handler, exercise it from a CLI right away:
```bash
ros2 service call /control/waypoints interfaces/srv/WaypointService '{...}'
```

---

## Issue 7 — Robot did not undock when Run Now fired while at the dock

### Symptom
After fixing Issue 6, Run Now loaded the route correctly. But when the
operator clicked the button while the robot was at the charge dock, the
state machine cycled IDLE → DOCKED → CHARGING repeatedly. The robot did
not undock and the route never executed.

### Why
The state machine's "I am docked, but I have a route, so I should
undock first" logic lives in `_handle_schedule_response()` (the
scheduler path). Our Run Now path goes through `_waypoint_service` and
calls `_start_navigation_directly()` directly — which assumes the robot
is already undocked. So the waypoints were loaded, but the state
machine kept evaluating "I'm at the charge position, must stay docked".

### Fix
`_waypoint_service` now checks the current state. If it's DOCKED or
CHARGING, it synthesises a response object the way the scheduler would,
and calls `_trigger_undocking_for_schedule(synth)`. That function:
1. Stores the geopath as `_pending_geopath`.
2. Transitions the state machine to UNDOCKING.
3. When UNDOCKING completes, the wp_follower picks up `_pending_geopath`
   and transitions to NAVIGATING.

Same path the scheduler uses → same safety gates apply.

### Lesson
When you add a new entry point (Run Now), make sure it goes through all
the **same state-machine dispatch** the existing entry points use. A
shortcut that "just starts navigating" will skip docked/charging
handling and confuse the state machine.

---

## Issue 8 — Roboclaw watchdog kept braking the motors

### Symptom
After all the above fixes, the wp_follower correctly entered NAVIGATING
and published `/cmd_drive` at 9.5 Hz with reasonable values. Robot
**still** did not move. Logs showed:
```
[roboclaw_drive]: [watchdog] no /cmd_drive in 1.04s > 0.3s — braking
```
…every 2-3 seconds, even though `/cmd_drive` was actively being
published.

### Why
The `roboclaw_wrapper_node` used a **single-threaded executor**. That
node has:
- A 10 Hz timer that reads battery voltage + motor currents over USB
  (slow — each read can take 100-300 ms).
- A `/cmd_drive` subscriber that writes motor duty over USB (fast).
- A 20 Hz watchdog timer that compares "now" against the last cmd_drive
  arrival time, and brakes if it's too long.

With one thread, the slow battery reads blocked everything else. The
watchdog timer fired late and saw an inflated "elapsed" → brake. The
cmd_drive callback ran late too, but the brake was already applied.
The motors never got commanded duty for long enough to actually move.

### Fix
1. `MultiThreadedExecutor(num_threads=3)` instead of `rclpy.spin(node)`.
2. `ReentrantCallbackGroup` on the voltage timer, drive_cmd_sub, debug
   sub, and watchdog timer → the executor can run them on different
   threads.
3. `threading.Lock` around every `self.rc.*` call (USB read + write).
   The lock protects the `/dev/ttyACM0` byte stream — concurrent reads
   and writes would garble it.

### Lesson
ROS 2 nodes default to a **single-threaded executor**. If you have one
slow callback and one fast callback that must not be starved, you need
`MultiThreadedExecutor` + `ReentrantCallbackGroup`. **And** you need a
mutex around any shared hardware resource (USB, GPIO) because now
callbacks can run in parallel.

Also: do not trust the watchdog's own log message about elapsed time
when the executor is jammed. The number it prints is inflated by the
delay in firing the timer itself.

---

## Issue 9 — Robot kept "trying to turn around" without making progress

### Symptom
With all earlier fixes in place, the robot finally entered NAVIGATING,
the cmd_drive cascade worked, and the robot did move. But the user
described: "robot is not following waypoints and just turn around."
`/cmd_drive` showed long bursts of opposite-sign wheel velocities
(robot commanded to spin in place), with little forward progress.

### Why
The waypoint controller uses the robot's **heading** (yaw) to decide
which way to turn. It read the heading from
`/fixposition/odometry_enu`'s **quaternion**. We then compared that
quaternion's yaw to the RTK ground-truth heading from
`/fixposition/ypr`:

| Time   | Truth (ypr)   | odom_enu quaternion |
|--------|---------------|--------------------|
| t=22s  |  +0.5°        | +138°              |
| t=30s  |  -1.0°        | -148°              |
| t=38s  |  -1.2°        | -178°              |

The quaternion is normalised (`|q|=1`) but encodes some attitude that
is **not** the robot's yaw — possibly the IMU-only attitude before RTK
heading fusion, or an unrelated body-frame convention. Either way, the
controller saw a heading that jumped randomly every iteration. So
"target direction minus my direction" → wildly different every loop →
the controller saturates to ±100 steering, alternates direction → robot
shakes back and forth without converging.

### Fix
Read the heading from the right source: `/fixposition/ypr`. This is the
moving-baseline GNSS heading from the dual-antenna RTK, stable to
about 1°.

1. Subscribe in `wp_follower`:
   ```python
   self.create_subscription(Vector3Stamped, '/fixposition/ypr',
                            self._ypr_callback, 10)
   ```
2. Cache the yaw:
   ```python
   def _ypr_callback(self, msg):
       self._last_yaw_rad = float(msg.vector.x)
   ```
3. In `_get_current_pose_map()`, after the pose is built, override
   `pose.pose.orientation` with a quaternion synthesised from
   `self._last_yaw_rad`. So all downstream consumers (linear waypoint
   follower, docking, undocking, obstacle avoidance) see the right
   heading.
4. Fall back to the old behaviour with a one-time warning if YPR has
   not arrived yet (avoids hard dependency at startup).

### Lesson
A normalised quaternion is **not** proof that it encodes what you
think. Always cross-check orientation against a known-good source
(GNSS heading, IMU integration, magnetometer). When you find a
mismatch, fix the producer if you can, but a "patch at the consumer"
override is often faster and safer.

---

## Issue 10 — My own first attempt at Issue 9 used the wrong field

### Symptom
After Issue 9's fix, the robot still spun without converging. The fix
had no effect.

### Why
The Fixposition message `/fixposition/ypr` is a `Vector3Stamped`. I
assumed `vector.z` was yaw (because in geometry conventions, yaw is
rotation around Z). But the Fixposition driver packs the fields in the
literal order **yaw / pitch / roll**:
```cpp
// fixposition_driver/data_to_ros2.cpp:221
// Euler angle wrt. ENU frame in the order of yaw pitch roll
msg.vector.set__x(enu_euler.x());  // ← YAW
msg.vector.set__y(enu_euler.y());  // pitch
msg.vector.set__z(enu_euler.z());  // ROLL
```
I read `vector.z` (= roll). Roll is ≈ 0° on flat ground, so the override
always wrote "yaw = 0" no matter how the robot was actually pointed.
The controller's symptom — saturated spin commands — looked identical
to "no fix applied".

### Fix
Change `msg.vector.z` → `msg.vector.x` in `_ypr_callback`. Verified on
live data: `vector.x = 1.335 rad ≈ 76.5°` when the robot was clearly
pointing NE.

### Lesson
**Never assume `Vector3` field semantics from the name `Vector3`.**
The fields x/y/z are just three numbers. The producer chooses what
they mean. When in doubt, read the publisher's source — in this case
a single grep into the driver code answered it.

---

## Issue 11 — Final run: success

### Symptom
After all 10 fixes, we re-ran the mission with route `mm` (3
waypoints). Robot drove for 104 s, then operator stopped it.

### Outcome

| Waypoint | Closest approach | Tolerance | Status |
|----------|------------------|-----------|--------|
| WP1      | 0.11 m           | 0.5 m     | reached |
| WP2      | 0.01 m           | 0.5 m     | reached (bullseye) |
| WP3      | 0.10 m           | 0.5 m     | reached |

Path quality: 49.7 m driven for a route whose straight-line length is
about 6-8 m. 75 % of the time the controller was rotating (which is
why the path was so long). That is a **separate** tuning problem in the
linear waypoint follower (`waypoint_follower.py`), not in the autonomy
plumbing. Left for the next session.

---

## Glossary (for next time you read code)

| Term | What it actually means here |
|------|----------------------------|
| **ENU** | East-North-Up. The map frame. yaw=0 means "facing East". yaw=+90° means "facing North". |
| **RTK** | Real-Time Kinematic — cm-accurate GPS using a base-station correction stream. |
| **RTK Fixed** | Status code `8`. The "green" you want for cm accuracy. |
| **RTK Float** | Status `5`. Dm accuracy. Orange in the UI. |
| **Dual-antenna heading** | The Fixposition has two GNSS antennas. The vector between them gives the robot's yaw directly, without needing motion. This is what `/fixposition/ypr.x` reports. |
| **TF tree** | ROS's coordinate-transform graph. `map → odom → base_footprint`. The wp_follower asks "where is base_footprint in map frame?" each loop. |
| **Watchdog** | A timer that fires periodically and resets motors to zero if `/cmd_drive` stops arriving. Safety against a crashed publisher. |
| **DutyAccel** | The Roboclaw command that ramps the motor duty cycle toward a target at a given acceleration. Range is roughly ±32767 = ±100 % duty. |
| **State machine** | The wp_follower's mode controller. Drives only in NAVIGATING / RETURNING_TO_HOME / UNDOCKING. Doesn't drive in IDLE / MANUAL / DOCKED. |
| **active_route** | The route currently loaded in the wp_follower. Empty until a Run Now or scheduler-fire injects waypoints. |

---

## How to debug this kind of problem yourself

A good order of triage when "the robot is not behaving":

1. **What does `/robot/state` say right now?**
   ```bash
   docker exec ros2_jetson bash -lc \
     "source /opt/ros/jazzy/setup.bash && \
      source /app/ros2_ws/install_ros2/setup.bash && \
      ros2 topic echo --field data /robot/state --once"
   ```
   Check `autonomous_enabled`, `autonomous_mode`, `active_route`,
   `charging_state`. If `active_route` is empty, the robot has nothing
   to do.

2. **What state-machine state is it in?**
   ```bash
   ros2 topic echo /tactical/robot/state --once
   ```
   `state` will be one of MANUAL / IDLE / DOCKED / CHARGING /
   UNDOCKING / NAVIGATING / etc.

3. **Is `/cmd_drive` being published, and what values?**
   ```bash
   ros2 topic echo /cmd_drive --once
   ```
   If `left_vel` and `right_vel` are zero or absent, the wp_follower
   is choosing not to drive (state machine in IDLE / MANUAL, or a
   safety gate fired).

4. **Are there warning logs?**
   ```bash
   docker logs ros2_jetson --since=2m 2>&1 \
     | grep -iE "warn|error|watchdog|stopping"
   ```

5. **Record a bag if it's not obvious.** Use `~/gits/diag_wp_record.sh`.
   Lays down all topics + a YAML snapshot of params for offline
   analysis.

Most autonomy failures fall in one of these buckets — and once you can
read the four signals above, you can diagnose any of the 10 issues in
this document on your own.

---

*Generated 2026-05-17 during the autonomy session that took the robot
from "doesn't move at all" to "reaches all 3 waypoints within 11 cm".
File alongside the existing RCA notes (`RCA_2026-05-15`, `RCA_2026-05-16`).*
