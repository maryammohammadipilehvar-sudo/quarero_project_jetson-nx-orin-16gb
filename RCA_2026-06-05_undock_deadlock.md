# RCA 2026-06-05 — Autonomy freezes after start: UNDOCKING deadlock

## Symptom
Operator starts autonomous mode; the robot "stops working suddenly" and never drives
the route.

## Live evidence (ros2_jetson, on-robot)
- State machine stuck in **UNDOCKING** (`/tactical/robot/state`), frozen across 20 s+.
- wp_follower control loop alive (10 Hz) but publishing `/cmd_drive = (0.0, 0.0)`
  every cycle — motors commanded to a dead stop, continuously.
- A schedule was active → on start the robot must undock before running the route.
- RTK healthy at the time: both GNSS `RTK_FIXED (8)`, fusion `init_status 2`,
  `/fixposition/fusion` solid at 10 Hz. (The `/fixposition/gnss1 status=2` seen via
  `ros2 topic echo` is the ROS `NavSatStatus` enum = GBAS, *not* Fixposition "DGPS=2"
  — a known trap; not the cause.)
- Robot position from live RTK: **1.28 m from home** (1.27 m laterally off the
  charge→home line) and **2.64 m from charge**. Line length ≈ 2.0 m.

## Root cause — two compounding defects
1. **Wrong completion semantic + no fail-safe (the freeze).**
   `UndockingState._execute_undocking` declared "Undocking complete" when it passed
   home *along the line projection* and stopped the motors, then called `on_undocked`
   → `transition_to(UNDOCKED)`. But the transition was gated (`can_exit_to` /
   `determine_next_state`) on **`is_near_home_pos` (0.4 m)**. When the reverse drive
   ended off-line (1.28 m from home), the gate vetoed the transition **every loop**,
   with no timeout or abort → permanent freeze commanding zero. Undocking's real goal
   is to *clear the dock* so the route can start (the follower drives to WP0 from
   wherever the robot is), not to land precisely on home.
2. **`_startup_at_charge_pos` lied "at charge" during undocking.**
   `_is_at_charge_position()` early-returned `True` whenever the startup latch was set
   (robot booted on the dock), regardless of GPS. So `is_at_charge_pos` stayed `True`
   while the robot was 2.6 m away, independently jamming the `not is_at_charge_pos`
   auto-transition clause.

Trigger: a large (~1.27 m) lateral excursion during the reverse undock. Contributing:
uncommitted branch changes raised `undock_speed_ratio 0.4→0.5` "for grass traction at
low battery" (battery 44 %). The veer is a *tuning* issue; the *freeze* is the design
defect — a tracking error must never become a silent hang.

## Fix (controller-authoritative completion + fail-safe; mirrors the docking
`on_dock_failed`/`FAILED` pattern already added on this branch)
`undocking_state.py`:
- Completion decided by **clearing the dock** (`distance_to_charge ≥
  undock_clearance_distance = 0.5 m`) OR genuinely reaching home — not by near-home.
- **`undock_timeout = 30 s`** and a "finished driving but still on the dock" check
  both route to a new `on_undock_failed` callback (latched, fires once).
- `can_exit_to(UNDOCKED)` / `get_valid_transitions` gate on **cleared charge**
  (`not is_at_charge_pos`) instead of near-home.

`tactical_wp_follower_node.py`:
- New `on_undock_failed(reason)`: zero motors, stop follower, publish German operator
  notice, `set_error()` (ERROR is operator-recoverable, exits only to IDLE/MANUAL).
- `on_undocked` now fails safe if its transition is somehow still rejected.
- `_startup_at_charge_pos` is dropped once the robot is in UNDOCKING/UNDOCKED so
  `is_at_charge_pos` stops lying.

**Not changed here (separate, on-robot-tuned):** the reverse line-following gains /
`undock_speed_ratio` that caused the lateral veer. The fail-safe makes the veer
non-fatal; retuning is a follow-up.

## Outcome guarantee
Every undock now reaches a terminal outcome: **NAVIGATING** (success, route starts) or
**ERROR + operator notice** (failure). It can no longer hang silently in UNDOCKING.

## On-robot test plan (operator present, hand on e-stop — CLAUDE.md §8)
Rebuild `ros2_jetson` only with the robot parked + manual mode active.
1. **Happy path:** dock the robot, start a schedule/route. Expect: backs out, clears
   the dock, transitions UNDOCKING→UNDOCKED→NAVIGATING, drives the route.
2. **Off-line finish (the bug):** nudge a lateral offset before starting (or replay the
   conditions). Expect: still completes (dock cleared) → route starts; no freeze.
3. **Failure path:** block the reverse path so it can't clear the dock. Expect: after
   ≤30 s → ERROR, motors zero, web shows "Abdocken fehlgeschlagen …", recoverable via
   MANUAL/IDLE. Verify it does NOT keep commanding zero in UNDOCKING.

## Immediate recovery (before the rebuild)
Put the robot in **MANUAL** (always wins over the deadlock), drive it back onto the
dock, then re-attempt. Re-arming autonomy without the fix re-enters the stuck undock.
