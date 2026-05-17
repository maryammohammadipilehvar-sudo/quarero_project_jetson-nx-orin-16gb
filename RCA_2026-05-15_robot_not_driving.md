# RCA — "robot not driving" after Nano → Orin NX swap

**Session:** 2026-05-15 evening, on the new Jetson (`quarero02@quarero02-desktop`, `192.168.10.226`).
**Status at sign-off:** unresolved. Operator interrupted the physical reseat ("ESP is currently disconnected"). To be continued the next day. **Customer demo is 2026-05-16** — see `HANDOFF.md`.

## 1. Symptom (operator-reported)

"Robot is not driving. Check ESP, Jetson, connectivity to PS5 controller." Operator clarified: the *only* change is moving the software stack from a **Jetson Nano 8 GB** carrier to a **Jetson Orin NX 16 GB** on the **NVIDIA P3768 Orin Nano Developer Kit carrier** (DTB `kernel_tegra234-p3768-0000+p3767-0000-nv.dtb`). Same wiring, same SoC peripherals, same software.

## 2. What was checked and confirmed healthy

| Item | Result |
|---|---|
| All 5 containers (`ros2_jetson`, `web_app`, `livox_ros2_jazzy`, `theramal_camera_jazzy`, `intel_realsense_ros2`) | Up |
| ROS nodes (`joy_controller`, `robot_controller`, `roboclaw_drive`, `tactical_wp_follower`, `tactical_scheduler`, `nav2_navigation_node`, etc.) | All running |
| Roboclaw 2x15A | Enumerated on `/dev/ttyACM0`, responding ("USB Roboclaw 2x15a v4.2.8") |
| `joy_controller` UART open | `/dev/ttyTHS1 @ 19200 baud` opened cleanly, no error |
| `/cmd_drive` subscribers | 1 (the drive node is listening) |
| 40-pin header pinmux | `config-by-function.py -l enabled` reports `uarta (8,10), i2c2 (27,28), i2c8 (3,5)` — UART is on pins 8/10 routed to `/dev/ttyTHS1`, identical to the Nano mapping |
| `nvgetty.service` | Masked (no console competing for `/dev/ttyTHS*`) |
| `dialout` group | Present for user and container |
| L4T / JetPack | R36.5.0 / JetPack 6.2 |
| DTB | Stock NVIDIA `p3768-0000+p3767-0000-nv.dtb` (matches the carrier) |

## 3. What was measured

- `ros2 topic hz /joy_drive_raw` → "topic does not appear to be published yet" (rate = 0 Hz).
- `joy_controller` log over ~38 min: exactly **one** `Invalid line` error and otherwise silent.
- Direct sniff `cat /dev/ttyTHS1` for 30 s with `ros2_jetson` stopped — **0 bytes**.
- Direct sniff `cat /dev/ttyTHS2` for 30 s — **0 bytes**.
- Re-sniff `/dev/ttyTHS1` for 45 s **while operator wiggled the ESP connector** — **361 bytes** captured. Pattern: long stretches of `\0` interrupted by short fragments that look like the **tail end of expected joy frames** (`6c\r\n`, `37f9\r\n`, `,6c57\r\n`). The ESP's frame format is `RX,RY,Right,Left,Select,Licht,GPS,CRC\r\n` per `joy_controller_node.py`.
- Post-reseat (operator paused mid-reseat) re-sniff for 20 s — **0 bytes** again.

## 4. Hypotheses considered

| # | Hypothesis | Status |
|---|---|---|
| H1 | Pinmux on the Orin NX dev-kit carrier defaults to GPIO instead of UART on pins 8/10 | **DISPROVEN** — `jetson-io` reports `uarta (8,10)` already enabled. |
| H2 | Wrong `/dev/ttyTHS*` index (Nano `ttyTHS1` ≠ Orin `ttyTHS1`) | **DISPROVEN** — both Nano and P3768 map header pins 8/10 to `/dev/ttyTHS1`. |
| H3 | Jetson Bluetooth service inactive blocks PS5 pairing | **N/A** — PS5 pairs to the **ESP32**, not the Jetson. |
| H4 | ESP unpowered or PS5 not paired | **DISPROVEN** — operator confirmed ESP power LED solid; PS5 lightbar solid (paired). |
| H5 | Intermittent physical contact at the 40-pin header connector | **WORKING HYPOTHESIS** — fit the wiggle-test pattern, but see §5 below. |
| H6 | Tegra Orin UART clock divider can't produce clean 19200 baud (would cause framing errors that appear as `\0` and only mid-frame bytes survive) | **NOT YET TESTED** — plausible alternative to H5. The fragment pattern (tail-only) also fits this. |
| H7 | **ESP deadman behavior: ESP only transmits while L1 is held on the PS5.** This is in operator gotchas (`HANDOFF.md:75`): *"ESP UART deadman: Jetson only sees Serial1 frames while L1 is held; silent UART ≠ broken wiring"*. | **DISCOVERED LATE — INVALIDATES PART OF THE DIAGNOSIS BELOW.** |

## 5. The big miss

I did not consult `HANDOFF.md` until late in the session. The "ESP only transmits while L1 is held" gotcha means:

- The 0-byte 30 s captures **could simply be the ESP being correctly silent** (L1 was never instructed to be held).
- The 361-byte wiggle capture **may not indicate intermittent contact** — the operator probably pressed buttons / triggers during the "press buttons / move sticks" portion of the test, including possibly L1 briefly.
- That said: the **fragment-only pattern** (tail-of-frame CRC bytes preceded by `\0` runs) is still not what a healthy ESP-with-L1-held link should look like. A healthy link would produce full lines like `RX,RY,Right,Left,Select,Licht,GPS,CRC\r\n` repeatedly at ~10 Hz. So H6 (baud/framing) is now competitive with H5 (intermittent contact) on remaining evidence.

I corrected the memory entries to reflect this; see `~/.claude/projects/-home-quarero02-gits/memory/`.

## 6. Tomorrow's plan (resume here)

**Pre-flight (operator):**
- Finish reseating the ESP cable on pins 8 (Jetson TX → ESP RX), 10 (Jetson RX → ESP TX), and a GND (any of 6/9/14/20/25/30/34/39).
- Confirm: ESP power LED solid, PS5 paired (solid lightbar).
- Confirm: no patrol scheduled, robot parked, e-stop in reach (CLAUDE.md §1 / §8).

**Test 1 — deadman-aware sniff.** Most important test.
1. `docker stop ros2_jetson`
2. `stty -F /dev/ttyTHS1 19200 raw -echo`
3. `timeout 20 cat /dev/ttyTHS1 > /tmp/ths1_L1.bin` while **operator holds L1 continuously**.
4. Inspect bytes. Expected if healthy: full lines, ~30 chars × ~10 Hz ≈ ~300 bytes/s = ~6 KB in 20 s.
5. `docker start ros2_jetson`.

**Branch on result:**
- **Full lines flow** → H7 was the whole story. Tell operator to drive normally; verify `/joy_drive_raw` rate at ~10 Hz while L1 held. **Done.**
- **Still fragments only, or still 0 bytes** → H7 alone is not enough. Continue.

**Test 2 — baud sweep.** If Test 1 fails the same way:
- Re-run a 10 s L1-held sniff at **9600, 38400, 57600, 115200** baud. If one of these produces clean frames, the Orin NX UART clock can't produce 19200 cleanly — fix is either reflash ESP firmware to a baud the Tegra produces cleanly, or live with reduced rate. (Less invasive: try `/dev/ttyTHS2` since UARTC may have a different clock parent.)

**Test 3 — UART loopback.** If Test 2 fails everywhere:
- Operator unplugs ESP cable from pins 8/10.
- Jumper pins 8 ↔ 10 with a wire.
- `echo -ne "HELLO\r\n" > /dev/ttyTHS1 &` then `timeout 3 cat /dev/ttyTHS1`. If "HELLO" returns → UART hardware fine, problem is ESP/cable. If nothing → carrier-side hardware fault.

## 7. Open side-issues (not blocking PS5 drive)

- **Fixposition RTK at `192.168.10.107` is unreachable.** `fixposition_driver_ros2` is in a crash loop: `Failed connecting to tcpcli://192.168.10.107:21000: No route to host`. Will block autonomous waypoint follower (RTK gating per CLAUDE.md §1). Need to power-cycle / re-network the Fixposition VRTK before any autonomous run. **Not blocking manual PS5 drive.**
- **`person_detection_bridge` polling 404s** continuously. Probably an unrelated misconfig of a backend URL; harmless for drive.
- `/dev/ttyUSB0` (CP2102) is mapped in the host but not referenced by any production node. Likely unused leftover. Don't worry about it.

## 8. State I left the system in

- No code changes. No config changes. No commits. No DT overlay applied.
- Containers running: `ros2_jetson`, `intel_realsense_ros2`, `livox_ros2_jazzy`, `theramal_camera_jazzy`, `web_app`.
- During the session I briefly `docker stop ros2_jetson` three times for direct UART sniffs and restarted each time. Final state: container is **up**.
- Capture artifacts: `/tmp/ths1.bin` (0 B), `/tmp/ths2.bin` (0 B), `/tmp/ths1_wiggle.bin` (361 B), `/tmp/ths1_v2.bin` (0 B). Tmpfs — will not survive reboot.
- ESP physical cable: per operator at sign-off, "currently disconnected — interrupted mid-reseat".

## 9. Memory entries written/updated this session

Under `/home/quarero02/.claude/projects/-home-quarero02-gits/memory/`:
- `MEMORY.md` (index)
- `project_platform_migration.md` — Nano → Orin NX context; corrected after pinmux hypothesis disproven.
- `reference_uart_chain.md` — PS5 → ESP32 → `/dev/ttyTHS1` → `joy_controller` → `/cmd_drive` → Roboclaw chain.

Still **missing from memory on this Jetson** (referenced by HANDOFF.md as living in older sessions but not migrated):
- ESP UART **deadman** gotcha (L1-held requirement) — I will add this now before signing off.
- PS5 fast-blink = ESP BT stack wedged; unplug/replug ESP.
- Roboclaw M2 reads ~0.5 A at idle (zero-offset, not real load).
- OAK-D Lite is named "realsense" in repo; depth topic `/oak/stereo/image_raw` (16UC1 mm); `pipeline_type=RGBD`, sync off.
- `customer_demo_2026_05_16.md` — referenced but not present.
