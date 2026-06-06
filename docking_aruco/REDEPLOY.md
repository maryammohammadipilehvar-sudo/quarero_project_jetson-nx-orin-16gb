# ArUco dock — re-deployment procedure (new site / markers moved)

Follow this whenever the robot moves to a new dock, or the markers are repositioned.
It takes ~5 minutes and removes the guesswork that caused the off-axis approaches.

**The one principle:** the GPS approach line must run **straight down the marker
normal, through the marker midpoint.** The line direction is fixed by the robot's
heading at the instant you save the charge point — so you must be **square + centred**
when you save. The `dock_align_check.py` tool makes that observable.

**Site-independent (DON'T re-tune at a new site):** all gains, tolerances, and
`invert_steering` in `aruco_dock.yaml` — they're properties of the robot, not the site.
**Site-specific (set every time):** the GPS `charge_point`/`home_point`, and
`stop_range_m` / `min_range_m`.

---

## 1. Mount the two markers
- DICT_4X4_50, **ID 0 and ID 1**, 150 mm black square each (files: `aruco_id0_150mm.png`,
  `aruco_id1_150mm.png`). Matte, glued **dead flat** to rigid board.
- **Coplanar**, in the dock face plane, **parallel to the dock face** (their shared
  normal = the approach axis).
- **~30 cm apart** centre-to-centre, side by side, at the **OAK lens height**.
- The **pair's midpoint on the dock centre-line** (where the robot sits when docked).

## 2. Teach the dock line (square + centred)
1. Manually drive the robot to the **perfect docked position** (contacts mated, centred
   between the markers). Come to a **full stop**.
2. Run the alignment tool inside `ros2_jetson`:
   ```
   docker exec -it ros2_jetson bash -lc \
     "source /opt/ros/jazzy/setup.bash && source /app/ros2_ws/install_ros2/setup.bash && \
      python3 /<repo>/docking_aruco/dock_align_check.py"
   ```
   (it needs **both** markers in view — it shows `cross`, `heading`, `perp`).
3. Adjust until it prints **`>>> SAVE NOW`** (cross < 3 cm AND heading < 3°, held 2 s).
   - To move sideways a diff-drive must turn — so after every nudge, **stop and let the
     numbers settle**; only the `SAVE NOW` state counts. Don't save mid-turn.
4. **Write down the `perp` value** it shows at `SAVE NOW` — that is your `stop_range_m`.
5. While the robot is **stationary**, **save the charge point in the web app.** The
   software sets `home_point` 2 m back along the (now-square) heading → line = marker normal.

## 3. Set the standoff in `routen/settings/aruco_dock.yaml`
- `stop_range_m:` = the `perp` value from step 2.4 (camera→marker range when docked).
- `min_range_m:` = `stop_range_m − 0.10` (hard anti-wall abort floor; must be below it).
- Leave everything else as-is.

## 4. Verify, then enable
1. Manually drive back ~1.5–2 m along the line; confirm **both** markers stay in view
   the whole way (`docker logs intel_realsense_ros2 --tail 3 | grep ARUCODIAG` →
   `seen=[0,1] mode=FUSED(2)`). If one drops out, move the markers slightly closer.
2. Set `enabled: true` in `aruco_dock.yaml` (picked up within 2 s).
3. **Supervised, e-stop in hand**, trigger a dock. Expect a smooth straight-in approach
   and `VISUAL dock complete`, nose square.

## 5. If it doesn't converge
- Re-run `dock_align_check.py` at the dock: if `cross`/`heading` aren't ~0 there, the
  teach is stale → re-do step 2 (most common cause).
- Watch the live trace: `docker logs ros2_jetson | grep "DOCKDIAG] servo"`.
  `cross` should shrink toward 0 as `perp` closes. If `cross` stays large, the line is
  off-axis → re-teach. The robot will **fail safe to GPS** rather than crash (anti-wall
  guard), so it won't hit the wall while you're calibrating.

---

### Safety invariants (do not weaken)
- `enabled: false` until step 4. First dock at any site is supervised with e-stop.
- `min_range_m` strictly below `stop_range_m`. The servo refuses to creep while off-axis
  (`align_cross_tol`) and stops short of the standoff (`creep_stop_margin`) so a bad
  teach fails to GPS instead of into the wall.
