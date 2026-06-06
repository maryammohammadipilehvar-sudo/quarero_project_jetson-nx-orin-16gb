# RCA 2026-05-31 — Docking reliability: spin-in-place + short-stop, and the robust fix

## Symptoms (observed live)
1. **Spins forever, never docks.** Robot entered DOCKING from RETURNING_TO_HOME ~2.4 m from
   the charge point, reached the line at the home end, entered ALIGN_TO_LINE, and then
   **rotated in place at near-max command without ever settling** (yaw swept the full circle
   monotonically; cmd_drive saturated at ±3.125 rad/s; no obstacle; RTK solid).
2. **Stops 10-15 cm short.** In earlier cycles the robot reached ~10-15 cm of the charge point
   then stalled, never closing the gap (parked residual measured 10.1 cm and 14.6 cm on two
   "correctly parked" runs — almost entirely longitudinal; lateral ≤1 cm, yaw ≤2.2°).

Constraint: **no contact-relay feedback** — the robot cannot sense that it is physically
connected to the charger. "Docked" must be inferred from RTK pose alone.

## Root causes
Live docking control is inline in `tactical_wp_follower_node` (delegates to
`DockingState`/`docking_state.py`). Heading is the Fixposition dual-antenna yaw (verified
correct — not stale, not the unreliable odom quaternion).

- **ALIGN spin:** the in-place rotation controller (`_compute_rotation_steering`) was
  **pure-P (`rotation_gain_d=0`)** targeting a **2° tolerance** — the worst-traction maneuver
  (in-place 90° pivot on grass) with no damping. It hunts/overshoots and never settles. A
  **2-10° min-steering dead band** let the command go too weak to break grass stiction, so
  rotation stalled and the **deadlock booster** ramped and jerked it past target. **No timeout**
  → spins forever.
- **Short-stop:** the final creep used `creep_speed_ratio=0.12` → ~3.4% duty
  (speed% → duty ≈ speed%·0.286), **below the drivetrain stiction floor** (~10% duty), so the
  robot physically cannot creep the last 10-15 cm.
- **Confirmation:** "docked" was GPS proximity at 5 cm (later 10 cm) with no model of where the
  robot actually rests, so it never confirmed at the true ~14 cm rest point.

## Changes (all in `docking_state.py` `DockingConfig` + logic, one line in wp_follower)
1. **ALIGN timeout fail-safe** (`align_timeout=12 s`): if ALIGN can't settle, hand off to
   FOLLOW_LINE, which drives toward the charge point while correcting heading (heading PD, with
   traction). Directly kills the infinite spin.
2. **ALIGN settles:** `alignment_tolerance` 2°→6°; `rotation_gain_d` 0→0.4 (damping);
   min-steering now applies for `|err| > alignment_tolerance` (closes the 2-10° dead band).
   FOLLOW_LINE cleans up residual heading dynamically.
3. **Creep reaches the dock:** `creep_speed_ratio` 0.12→0.35 (≈10% duty) so the final creep
   actually moves on grass and seats into the dock.
4. **Seated confirmation (no contact sensor):** `charge_position_tolerance` 0.10→0.18 (covers the
   measured ~14.6 cm rest residual + margin); lateral stays tight at 3 cm (anti-fake-success);
   confirm **only once the robot stops getting closer** (within 0.5 cm of its closest approach =
   seated against the dock) over `docked_confirm_readings` (3). wp_follower
   `_charge_position_tolerance_charging` 0.10→0.18 to keep `is_at_charge_pos` consistent.

## On-robot test plan (operator present, e-stop in reach — CLAUDE.md §8)
- Rebuild `ros2_jetson` (robot parked + manual). Re-enable autonomous.
- Trigger a dock (Run-Now a short route → return, or go-to-charge). Watch
  `/tactical/robot/charging_status` and `docker logs ros2_jetson | grep -i dock`.
- **Expect:** ALIGN settles within ~6° (or times out at 12 s and hands to FOLLOW); creep drives
  in without stalling; "Docking complete - seated at charge position" at ~10-18 cm, lateral ≤3 cm.
- **Tuning knobs if needed:** `creep_speed_ratio` (raise if still stalls / lower if overshoots),
  `align_timeout`, `alignment_tolerance`, `rotation_gain_d`, `charge_position_tolerance`.
- After each dock, run `ros2_ws/pose_diff_measure.py` to log the residuals.

## Residual risks
- No contact sensor remains: "seated" is a GPS proxy (no-further-progress). If the dock is not a
  hard physical stop, the robot may seat slightly short — acceptable, never a false "beside the
  plates" success (lateral gate holds).
- Creep at 0.35 is firmer than before; verify it doesn't shove the dock. Lower if so.
