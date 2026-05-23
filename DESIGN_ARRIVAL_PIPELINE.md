# DESIGN — Arrival-Detection / Notify / 72h-Retention Pipeline

**Status:** DRAFT — awaiting operator approval. No code is written until this doc is approved (per the contract we set in conversation).

**Audience:** the operator. Read this end-to-end before any of `~/gits/` is touched. Companion to `CODEBASE_MENTAL_MODEL.md`.

---

## 1. Goal

When a previously-absent **person or vehicle** appears in the Eneo camera FOV, the system:

1. Confirms it's a real arrival (not a single-frame false positive, not the same object re-appearing within minutes).
2. **Records** pre/post-event video clips to disk.
3. **Phones the operator** with a pre-recorded voice message and requires DTMF acknowledgement.
4. **Auto-deletes** the captured media after **72 hours**, with a retained audit log proving deletion (no PII).

"Industrial-grade" here means: false-positive rate low enough that the operator won't mute it, no silent failures (a missed call is louder than no call), GDPR-defensible deletion, all components independently testable and reversible.

## 2. Scope

**In scope (this doc):**
- New detector on Jetson 2 alongside the existing `live_detect.py` (untouched).
- New event ingest endpoint on Jetson 1 (`web_app`).
- Re-enabling `video_ringbuffer` (AUDIT HIGH-2).
- Telephony adapter with provider abstraction (Twilio first, Sipgate later).
- Retention timer + deletion audit log.

**Out of scope (call out explicitly):**
- Authenticating the broader web API (HIGH-3) — the new `/api/security/arrival` endpoint *will* be auth-gated, but the rest of `:8020` stays unauth'd until a separate hardening pass.
- Rotating the hardcoded Eneo RTSP credentials (HIGH-4) — note the credential is now in *three* places: thermal_stream on Jetson 1, live_detect.py on Jetson 2, and (soon) arrival_detection.py on Jetson 2. Recommend moving all three to env var in a follow-up.
- Face / vehicle re-identification across long absences (we use simple track IDs from one stream).
- Multi-camera correlation (we only watch the Eneo `camera=2` stream).
- DPIA, customer consent forms, on-site signage — legal/procurement, not code.
- Alarm and siren hardware wiring (HIGH-5 is a separate task).

## 3. Success criteria

The design is successful when:

- [ ] A person walking into FOV from off-camera triggers exactly one phone call within 5 seconds.
- [ ] The same person walking out and back within 10 minutes does **not** re-trigger.
- [ ] A vehicle driving into FOV triggers exactly one call.
- [ ] The operator's phone rings, plays a pre-recorded message, and accepts a "press 1" acknowledgement; if not acknowledged, retries once after 30 s, then writes a failure log + email.
- [ ] An MP4 clip and a JSON metadata file land in `/routen/security_events/{event_id}/`.
- [ ] Exactly 72 hours after creation, the clip and metadata are deleted, the index entry is removed, and the deletion is logged.
- [ ] Disabling any one of the four new systemd services (arrival-detection on J2, retention timer on J1, ringbuffer on J1, telephony) does not break the others or the existing autonomy stack.

## 4. Architecture

```
                                Eneo @ 192.168.10.193 (RTSP, camera=2)
                                              │
                ┌─────────────────────────────┼──────────────────────────┐
                ▼                             ▼                          ▼
       [EXISTING — UNTOUCHED]         [NEW on Jetson 2]      [EXISTING thermal_stream
       live_detect.py                  arrival_detection.py    on Jetson 1, unchanged]
       Person-only YOLO                 Multi-class YOLO
       MJPEG :8080                      + ByteTrack
       (operator's live view)           + dwell confirmation
                                        + per-track cooldown
                                              │
                                              │ HTTP POST (with auth token)
                                              │ {event_id, ts, class, track_id, bbox,
                                              │  frame_jpeg_base64, source}
                                              ▼
                              ┌──────────────────────────────────────┐
                              │  Jetson 1  (192.168.10.226)          │
                              │                                       │
                              │  web_app : 8020                       │
                              │   POST /api/security/arrival          │
                              │   ────────────────────────────        │
                              │   1. Validate token, payload          │
                              │   2. Generate event_id                │
                              │   3. Call /capture_event_clips        │
                              │      (ROS service from ringbuffer)    │
                              │   4. Write metadata.json              │
                              │   5. Append to events_index.json      │
                              │   6. Enqueue telephony call           │
                              │                                       │
                              │  video_ringbuffer (ROS, RE-ENABLED)   │
                              │   ─ subscribes /ip_camera/rgb_raw     │
                              │   ─ keeps 10s pre + 10s post buffer   │
                              │   ─ on capture → saves MP4 to         │
                              │     /routen/security_events/{id}/     │
                              │                                       │
                              │  notification_dispatcher (NEW)         │
                              │   ─ HTTPS POST → ntfy.sh/{topic}      │
                              │   ─ priority=max, ring through silent │
                              │   ─ attaches frame.jpg + Ack action   │
                              │   ─ 30 s ack timeout → re-fire louder │
                              │   ─ on hard fail: SMTP to operator    │
                              │                                       │
                              │  retention_service (NEW systemd timer)│
                              │   ─ runs hourly                       │
                              │   ─ scans /routen/security_events/    │
                              │   ─ deletes any event with            │
                              │     created_at < now - 72h            │
                              │   ─ writes deletion_audit.log         │
                              └──────────────────────────────────────┘
```

## 5. Component specs

### 5.1 `arrival_detection.py` (Jetson 2, NEW)

**Path:** `/home/quarero-00/Desktop/camera-ai/arrival_detection.py` (alongside the untouched `live_detect.py`).

**Systemd unit:** `/etc/systemd/system/arrival-detection.service`, identical pattern to `person-detection.service`. Independent; restart loop on its own; can be `systemctl disable`d without touching the live person stream.

**Pipeline:**

```python
# Pseudo-code; real impl will be ~250 lines
COCO_CLASSES_OF_INTEREST = {
    0: "person",
    2: "car",
    3: "motorcycle",
    5: "bus",
    7: "truck",
}

CONFIG = {
    "rtsp_url": "rtsp://Quarero:quarero00@192.168.10.193:554/axis-media/media.amp?camera=2&audio=0",
    "model_path": "yolov8n.engine",          # reuse the existing TensorRT engine
    "inference_fps": 5.0,
    "confidence_threshold": 0.55,
    "dwell_frames": 10,                       # 10 frames @ 5 FPS = 2.0 s minimum dwell
    "track_expiry_s": 60.0,                   # track is "gone" after 60 s unseen
    "per_track_cooldown_s": 600.0,            # don't re-fire same track ID for 10 min
    "post_url": "http://192.168.10.226:8020/api/security/arrival",
    "post_token_env": "ARRIVAL_INGEST_TOKEN",
    "frame_jpeg_quality": 70,
}

# Loop:
#   1. Read RTSP frame
#   2. YOLOv8 inference (multi-class, classes filtered to COCO_CLASSES_OF_INTEREST)
#   3. ByteTrack update with detections → list of (track_id, class, bbox, score)
#   4. For each track:
#      - dwell_counter[track_id] += 1
#      - If dwell_counter[track_id] == dwell_frames AND track_id not in cooldown:
#          fire_arrival_event(track_id, class, bbox, current_frame_jpeg)
#          mark track_id cooled-down for per_track_cooldown_s
#   5. Expire tracks not seen for track_expiry_s; their cooldown remains in effect
```

**Failure handling:**
- RTSP disconnect → exponential backoff reconnect, same pattern as existing `live_detect.py`.
- POST to Jetson 1 fails → buffer up to 100 pending events in memory; retry with 1-min, 5-min, 15-min backoff. **Do not block inference on POST**. If buffer overflows, drop oldest (with a counter log).
- ByteTrack lib unavailable → service fails to start (loud, not silent — systemd journal entry).

**Resource budget:** YOLOv8n at 5 FPS uses ~3% CPU + ~600MB GPU (already proven by `person-detection.service`). Running a second copy = ~6% CPU + ~1.2GB GPU. Jetson 2 has ample headroom (currently ~1.5GB used, ~14GB free).

### 5.2 Arrival event HTTP protocol

**Endpoint:** `POST http://192.168.10.226:8020/api/security/arrival`

**Auth:** `Authorization: Bearer <token>` where token is set via env var `ARRIVAL_INGEST_TOKEN` on both sides. Generated once, stored in:
- Jetson 2: `/home/quarero-00/Desktop/camera-ai/.env` (chmod 600, not committed)
- Jetson 1: docker-compose `env_file` for `web_app` container

**Request body (JSON):**
```json
{
  "version": 1,
  "source": "jetson-ai/arrival_detection",
  "timestamp": "2026-05-23T13:42:18.443Z",
  "track_id": 47,
  "class_label": "person",
  "class_id": 0,
  "confidence": 0.81,
  "bbox": {"x1": 234, "y1": 102, "x2": 411, "y2": 538},
  "frame_jpeg_base64": "<base64-encoded 640x360 JPEG, ~30KB>",
  "camera": "eneo_rgb_cam2"
}
```

**Response (200):**
```json
{
  "accepted": true,
  "event_id": "evt_20260523T134218_47",
  "actions": ["clip_capture_queued", "telephony_queued"]
}
```

**Error responses:** 401 (bad token), 400 (malformed), 503 (telephony unavailable but event still stored).

**Idempotency:** server keys on `(source, track_id, ts_minute)` — duplicate POSTs within same minute return the same event_id without re-triggering capture/call. Protects against retry-after-success.

### 5.3 `/api/security/arrival` handler (Jetson 1 web_app, NEW)

**File:** `tactical_mower/ros2_ws/src/robot_web_interface/src/robot_web_interface/api/routes/security_arrival.py` (new module, sibling of existing `routes/*.py`).

**Hook into RobotNode** (in `main.py`): call `init_security_arrival_router(node)` in lifespan (same pattern as `init_routes_router` from RCA_2026-05-17 Issue 5).

**Handler flow:**
1. Validate Bearer token (constant-time compare). Reject with 401 if mismatch.
2. Validate payload (Pydantic model). Reject with 400 if malformed.
3. Compute `event_id = "evt_" + ts.strftime("%Y%m%dT%H%M%S") + "_" + str(track_id)`. Check idempotency cache.
4. Create directory `/routen/security_events/{event_id}/`.
5. Decode `frame_jpeg_base64`, save as `frame.jpg` in event dir.
6. Build `metadata.json`:
   ```json
   {
     "event_id": "evt_...",
     "created_at": "2026-05-23T13:42:18.443Z",
     "expires_at": "2026-05-26T13:42:18.443Z",
     "source": "jetson-ai/arrival_detection",
     "class_label": "person",
     "track_id": 47,
     "confidence": 0.81,
     "bbox": {...},
     "camera": "eneo_rgb_cam2",
     "clip_status": "pending",
     "telephony_status": "pending"
   }
   ```
7. Call ROS service `/capture_event_clips` (video_ringbuffer) — async, update `clip_status` in metadata on callback.
8. Enqueue telephony job (background task) — async, update `telephony_status` on completion.
9. Append `{event_id, created_at, class_label}` to `/routen/security_events/events_index.json` (atomic write via temp+rename).
10. Return 200 with event_id.

### 5.4 video_ringbuffer re-enable (AUDIT HIGH-2)

**Today:** `tactical_mower/ros2_ws/src/system_bringup/launch/controller.launch.py:64-78` has the entire `video_ringbuffer_node = Node(...)` block commented out. Re-enable means:

1. Uncomment the block.
2. Wire up parameters to read from `settings.yaml.security_event_defaults`:
   - `storage_root: /routen/security_events`
   - `pre_event_seconds: 10.0`
   - `post_event_seconds: 10.0`
   - `max_total_size_mb: 20480.0`
   - `max_buffer_memory_mb: 1024.0`
3. Add `video_ringbuffer_node` to the returned LaunchDescription.
4. Verify the `/capture_event_clips` service is advertised after restart.
5. Verify the camera topics it subscribes to actually publish (per `eneo_event_publisher.eneo_event_node.py:20-24`, `EVENT_RECORDING_CAMERA_IDS = ['eneo_rgb', 'eneo_thermal']` — confirm those topics exist).

**Manual test before integration:**
```bash
docker exec ros2_jetson ros2 service call /capture_event_clips \
  interfaces/srv/CaptureClips "{event_id: 'manual_test_001'}"
ls -la /home/quarero02/gits/routen/security_events/manual_test_001/
# Expect: eneo_rgb.mp4 (~20 s long), eneo_thermal.mp4
```

**Operator caveat:** this restarts `ros2_jetson` → robot becomes briefly unavailable. Schedule during a non-patrol window. CLAUDE.md §3.

### 5.5 Notification dispatcher (NEW) — ntfy.sh

**Decision:** real telephony (Twilio/Sipgate) was rejected — Twilio's signup is failing for this operator and Sipgate costs money per call. Replaced with **ntfy.sh push notifications** (free forever, no signup, no credit card, rings through silent mode). Trade-off: the operator must install the **ntfy** app on their phone (free, available for Android, iOS, F-Droid, web). If the app is installed and the phone has network, behaviorally this is *louder and more reliable* than a phone call (priority-max bypasses DND).

**File:** `tactical_mower/ros2_ws/src/robot_web_interface/src/robot_web_interface/services/notifications.py`

**Provider abstraction (keeps door open for future telephony swap):**
```python
class NotificationProvider(Protocol):
    def send(self, alert: ArrivalAlert) -> NotificationResult: ...

class NtfyProvider:           # default; HTTPS POST to ntfy.sh
    ...

class SipgateProvider:        # stub for later — if customer ever requires a real call
    ...

class TelegramProvider:       # stub for later
    ...

class StdoutProvider:         # dev/test mode: logs what would be sent
    ...
```

**Configuration — env vars (never committed):**
- `NOTIFICATION_PROVIDER` — `ntfy` (default) | `sipgate` | `telegram` | `stdout`
- `NTFY_SERVER` — default `https://ntfy.sh` (the free public server). Override to your own URL when self-hosted.
- `NTFY_AUTH_TOKEN` — optional; only used when the topic is private or when self-hosted ntfy has auth enabled.

**Operator-editable runtime values** (in `/routen/settings/settings.yaml`, editable via web UI per §5.7):
- `topic` — the ntfy topic name. **Auto-generated as a 32-char random string** on first install (e.g. `quarero-robot-a7k3p9j2x4m8…`); operator can rotate via UI. Treat as a secret — anyone who knows the topic can post AND subscribe (this is how ntfy's free tier works without auth).
- `enabled` — master kill-switch.
- `quiet_hours_from` / `quiet_hours_to` — optional HH:MM strings. During the window, **notifications are suppressed but events are still captured + logged + emailed**. Empty = no quiet hours, ring 24/7.
- `ack_timeout_seconds` — default 30. If no Ack action click within this window, re-fire one louder reminder (higher priority + `tags=warning,bell`).
- `max_retries` — default 1.

**Notification payload (POST `https://ntfy.sh/{topic}`):**
```http
POST /quarero-robot-a7k3p9j2x4m8… HTTP/1.1
Host: ntfy.sh
Title: 🚨 Sicherheitswarnung — Person erkannt
Priority: max
Tags: rotating_light,warning
Attach: https://<NETBIRD-IP>:8020/api/events/evt_20260523T134218_47/video/frame
Click: https://<NETBIRD-IP>:8020/events?focus=evt_20260523T134218_47
Actions: http, Bestätigen, https://<NETBIRD-IP>:8020/api/security/events/evt_…/ack, method=POST, headers.Authorization=Bearer <ACK_TOKEN>; \
         http, Roboter anhalten, https://<NETBIRD-IP>:8020/api/control/emergency_stop, method=POST, headers.Authorization=Bearer <ACK_TOKEN>, body={"state":true}
Content-Type: text/plain

Person oder Fahrzeug erkannt am 2026-05-23 13:42:18.
Standort: <site name from settings>.
Konfidenz: 81%. Track-ID: 47.
```

**Behavior:**
- `Priority: max` → on Android, rings even in silent/DND. On iOS, requires the operator to grant "critical alerts" permission once.
- `Attach:` URL → the frame snapshot appears as an image preview in the notification, so the operator sees *what* triggered it without opening the app.
- `Click:` → tapping the notification opens the web UI focused on this event (clip playback).
- `Actions:` → two inline buttons in the notification:
  1. **Bestätigen** (Acknowledge) — one-tap POST to mark the event ack'd. No follow-up notification fires.
  2. **Roboter anhalten** (Emergency stop) — one-tap e-stop. Useful if the operator sees something serious and wants to stop the robot from the notification without opening any UI.
- If no Ack click within `ack_timeout_seconds` (30 s default) → re-fire with even higher attention (additional reminder). After `max_retries` → SMTP fallback to `settings.yaml.security_email.recipients` and metadata marked `notification_status=failed`.

**SMTP fallback (kept per operator request):**
- Always fires during quiet hours (instead of suppressing entirely).
- Fires when ntfy delivery fails (no Ack after retries).
- Uses the existing `settings.yaml.security_email.*` config (already wired but currently `enabled: false`).
- Includes frame.jpg as attachment + link to clip.

**Topic onboarding (zero-friction):**
- On first launch, system generates a random topic name and saves to `settings.yaml`.
- Web UI Settings page shows the topic name + a **QR code** (generated server-side using `qrcode` library) encoding `ntfy://ntfy.sh/{topic}`.
- Operator installs the ntfy app, taps "+", scans the QR code, done. No account.
- If the operator wants to rotate the topic (e.g. they shared the QR by mistake), one click in the UI generates a new random topic and shows a new QR.

### 5.6 Retention service (NEW)

**File:** `tactical_mower/scripts/retention_service.py` (standalone script; no ROS deps — pure Python).

**Systemd timer:** `/etc/systemd/system/retention-service.{service,timer}` — hourly, calls `retention_service.py --purge`.

**Operation:**
```python
NOW = datetime.now(timezone.utc)
RETENTION_HOURS = 72  # from settings.yaml if we want to make configurable later

for event_dir in (ROOT / "security_events").iterdir():
    if not event_dir.is_dir():
        continue
    metadata = json.loads((event_dir / "metadata.json").read_text())
    created = datetime.fromisoformat(metadata["created_at"])
    age = (NOW - created).total_seconds() / 3600
    if age >= RETENTION_HOURS:
        # 1. Remove from events_index.json (atomic)
        # 2. shutil.rmtree(event_dir)
        # 3. Append to deletion_audit.log (append-only):
        #    {now, event_id, class_label, age_hours, files_deleted_count}
```

**`deletion_audit.log` location:** `/routen/security_events/deletion_audit.log`. JSONL format, append-only, never rotated by us (operator's log policy). Contains no PII (no bbox, no frame, no track ID — only event_id, timestamp, and what was deleted). Indefinite retention is the GDPR-required proof.

**Dry-run mode:** `retention_service.py --dry-run` lists what *would* be deleted. Used for testing.

**Atomic safety:** if power dies mid-delete, on next run any orphaned `metadata.json` without a `frame.jpg`/`clip.mp4` is treated as already-deleted and removed from the index. No double-delete in the audit log (deduped by event_id within a 60 s window).

### 5.7 Web UI settings extensions (NEW)

The operator must self-serve: install the ntfy app, scan a QR code to subscribe, and control the alerting behavior. Add a new "Sicherheitsbenachrichtigung" section to `static/settings.html` + matching backend handlers.

**New form section "Sicherheitsbenachrichtigung":**

| Field | Type | Backed by |
|---|---|---|
| ntfy Topic | text input (read-only display) + "Rotieren" button | `settings.yaml > security_arrival.notifications.topic` |
| QR-Code | image rendered server-side; operator scans with ntfy app | derived from `topic` + `NTFY_SERVER` |
| Benachrichtigungen aktiviert | checkbox | `settings.yaml > security_arrival.notifications.enabled` |
| Ruhezeit von | time input (HH:MM, optional) | `settings.yaml > security_arrival.notifications.quiet_hours_from` |
| Ruhezeit bis | time input (HH:MM, optional) | `settings.yaml > security_arrival.notifications.quiet_hours_to` |
| Test-Benachrichtigung senden | button | `POST /api/security/notifications/test` — fires one test push to the current topic |
| SMTP-Fallback aktiviert | checkbox | `settings.yaml > security_email.enabled` |
| SMTP-Empfänger (komma-getrennt) | text input | `settings.yaml > security_email.recipients` |

**New endpoints (auth-gated):**
- `GET /api/settings/notifications` → `{topic, enabled, quiet_hours_from, quiet_hours_to, qr_png_base64}`
- `POST /api/settings/notifications` body `{enabled, quiet_hours_from, quiet_hours_to}` — never accepts a custom topic via this endpoint (preserves randomness)
- `POST /api/settings/notifications/rotate-topic` — generates a new random 32-char topic, writes it, returns the new QR
- `POST /api/security/notifications/test` — sends one test push using current topic; returns success/failure

**Validation rules:**
- Topic, if regenerated, is always server-generated random (the UI never accepts user input for it — prevents weak topic names).
- `quiet_hours_from` / `quiet_hours_to` either both set (HH:MM) or both empty. Crossing midnight (`23:00` → `06:00`) is supported.
- If `enabled=true` and `topic` is empty → server auto-generates a topic on the spot (operator can't lock themselves out by checking the box before generating a topic).
- All settings changes append one line to a config-change audit log (`/routen/settings/changes.log`) — no PII, just `{ts, field, old_value_hash, new_value_hash}`.

**The QR flow is the killer feature** — operator goes from "I want alerts" to "phone buzzing" in under 60 seconds with zero account creation:
1. Click "Show QR" in Settings.
2. Install ntfy from Play Store / App Store.
3. Scan QR in ntfy app's "Subscribe to topic" screen.
4. Click "Test-Benachrichtigung senden" — phone buzzes.
5. Done.

## 6. Data model summary

| File | Content | Lifetime |
|---|---|---|
| `/routen/security_events/{event_id}/metadata.json` | event metadata, ack status, statuses | 72h |
| `/routen/security_events/{event_id}/frame.jpg` | snapshot at moment of arrival | 72h |
| `/routen/security_events/{event_id}/eneo_rgb.mp4` | 20s clip from ringbuffer | 72h |
| `/routen/security_events/{event_id}/eneo_thermal.mp4` | 20s clip from ringbuffer | 72h |
| `/routen/security_events/events_index.json` | one-line-per-event index for the web UI | trimmed in sync |
| `/routen/security_events/deletion_audit.log` | append-only proof of deletion | **indefinite** |

`event_id` format: `evt_<YYYYMMDDTHHMMSS>_<track_id>`. Globally unique within Jetson 2's reboot lifetime (track IDs reset on detector restart; the timestamp suffix prevents collision across restarts).

## 7. Configuration additions

**`/routen/settings/settings.yaml`** — new section:
```yaml
security_arrival:
  enabled: true
  jetson2_source: "jetson-ai/arrival_detection"
  dwell_seconds: 2.0
  confidence_threshold: 0.55
  per_track_cooldown_minutes: 10
  retention_hours: 72
  notifications:
    enabled: false              # operator turns on via UI after generating topic
    provider: "ntfy"            # ntfy | sipgate | telegram | stdout (env overridable)
    topic: ""                    # auto-generated random 32-char string on first enable
    quiet_hours_from: ""         # HH:MM optional (e.g. "23:00")
    quiet_hours_to: ""           # HH:MM optional (e.g. "06:00")
    ack_timeout_seconds: 30
    max_retries: 1
# existing security_email.* section reused for SMTP fallback — enable it via UI
```

**Default state is safe:** `notifications.enabled=false` and `topic=""` ship by default. The system records events and clips from day one but won't send any notification until the operator (a) clicks "generate topic", (b) scans the QR with their phone, (c) flips the enable switch in the UI.

**Env vars** (never in repo):
- Jetson 2: `ARRIVAL_INGEST_TOKEN`
- Jetson 1 `web_app` container: `ARRIVAL_INGEST_TOKEN` (same value), `NOTIFICATION_PROVIDER` (default `ntfy`), `NTFY_SERVER` (default `https://ntfy.sh`), `NTFY_AUTH_TOKEN` (optional)

**Env vars** (never in repo):
- Jetson 2: `ARRIVAL_INGEST_TOKEN`
- Jetson 1 `web_app` container: `ARRIVAL_INGEST_TOKEN` (same value), `TWILIO_ACCOUNT_SID`, `TWILIO_AUTH_TOKEN`, `TWILIO_FROM_NUMBER`

## 8. Failure modes & mitigations

| Failure | Detection | Mitigation |
|---|---|---|
| Jetson 2 → Jetson 1 network drops | POST failure, in-memory buffer fills | Backoff retry; buffer caps at 100 events; oldest dropped with metrics |
| Jetson 1 web_app crashes during call | event metadata stuck in `telephony_status=pending` | Hourly sweeper marks pending events older than 1h as `failed` and re-queues once |
| Twilio account out of credit | API returns 402 | telephony_status=failed; SMTP fallback fires; web UI shows credit warning |
| Ringbuffer unavailable when capture called | `/capture_event_clips` returns error | `metadata.json.clip_status=failed`; event still recorded with `frame.jpg` (which is captured by the detector independently, not by the ringbuffer) |
| Detector firing false positive every frame | per-track cooldown limits to 1 call / 10 min per track; per-recipient cooldown limits to 1 call / 60 s overall | logs counter for operator review; tunable threshold |
| Retention timer skipped (Jetson off) | next run catches up — purges everything that's now >72h | hourly cadence means worst-case 1h late, not 24h late |
| Power loss mid-write | tmp-and-rename for index; rmtree for events handles partial state | dedup window in audit log |
| Token leaked | anyone on LAN can POST fake arrivals | token rotatable; future: mTLS between J2 and J1 |

## 9. Security & GDPR

**Token auth** is the minimum bar. Stronger options later: mTLS (Jetson 2 ↔ Jetson 1 both have certs), or move the POST to a Unix socket if/when the two services collapse onto one machine.

**GDPR posture:**
- Lawful basis: customer's legitimate interest (site security) + customer's data processing agreement with you.
- Data minimisation: only frames containing detections are stored; the full RTSP stream is not persisted.
- Retention: 72h hard cap, automated deletion, audit log retained as compliance proof.
- Right to erasure: operator can manually delete an event via existing `DELETE /api/events/{id}` (already implemented); the audit log records the manual deletion same as automatic.
- Subject access requests: the audit log + the index allow point-in-time recovery of "what we held when".
- Cross-border: Twilio is US; if customer objects → swap to Sipgate (EU). Architecture supports this.

**Customer-facing requirements (not code, but flag to operator):**
- Site signage required ("Videoüberwachung — Aufnahmen werden 72h gespeichert").
- DPIA recommended for the customer's records.
- Customer should sign off on the recipient phone number being notified.

## 10. Test plan

### 10.1 Unit-level (no robot needed)
- `arrival_detection.py` tracker: feed a synthetic sequence of bboxes, assert correct track IDs assigned, correct fire-on-dwell, correct cooldown.
- Telephony adapter: `StdoutProvider` test — verify call payload is correctly formed.
- Retention: build a fake `security_events/` tree with mixed ages, run with `--dry-run`, assert only >72h items listed.
- Web endpoint: pytest with `httpx`, hit `/api/security/arrival` with valid + invalid tokens + idempotency dupe; assert correct responses.

### 10.2 Integration on robot (operator present, robot parked, e-stop in reach — CLAUDE.md §1 §8)
- **Test A: clip capture path.** Manually call `/capture_event_clips` after ringbuffer re-enable. Verify MP4s land in `security_events/manual_test_001/`. Verify clip plays in web UI.
- **Test B: detector → event POST.** Walk into camera FOV. Verify `arrival_detection.service` logs an arrival, Jetson 1 receives the POST, event dir created, metadata correct.
- **Test C: telephony.** Operator's actual phone rings, plays MP3, accepts "press 1". Verify `metadata.telephony_status=acknowledged`. Verify retry on no-answer.
- **Test D: no-spam.** Walk back and forth across FOV 10 times in 2 minutes. Verify exactly 1 call.
- **Test E: vehicle.** Drive a car through FOV. Verify event fires with `class_label=car`.
- **Test F: retention.** Wait 72h (or manually adjust an event's `created_at` to 73h ago). Verify next retention run deletes it, audit log appended, index updated. Verify deleted event no longer appears in web UI.

## 11. Rollout plan

**Phase 1** — `video_ringbuffer` re-enable + verify (Test A). This is the AUDIT HIGH-2 fix and is independently valuable.

**Phase 2** — `arrival_detection.py` on Jetson 2 with `StdoutProvider` for telephony (just logs to journal). No real calls. Verify detection accuracy with operator in FOV.

**Phase 3** — `/api/security/arrival` endpoint + clip capture wiring. Still no real calls.

**Phase 4** — Twilio adapter wired in, account set up. Switch from `StdoutProvider` to `TwilioProvider`. Real call test (Test C).

**Phase 5** — Retention timer enabled (after 24-48h of accumulated test events). Initial run in `--dry-run` mode; operator confirms list looks right; then live.

**Phase 6** — End-to-end test (Tests B, D, E, F).

Each phase is independently reversible: `systemctl disable` the new service, comment out the launch line, etc. Nothing in this design touches the autonomy stack or the existing `live_detect.py`.

## 12. Open questions for operator

**All resolved in conversation:**
- ~~**Operator phone number**~~ → not used; ntfy push notifications instead. Operator subscribes by scanning a QR code in the web UI Settings page. No phone number ever leaves the robot.
- ~~**Voice message**~~ → not used. Notification carries German title + body + frame.jpg preview + Bestätigen/E-Stop action buttons. (See §5.5 payload example.)
- ~~**Camera source**~~ → Eneo at `rtsp://Quarero:quarero00@192.168.10.193:554/axis-media/media.amp?camera=2&audio=0`.
- ~~**Quiet hours**~~ → operator-configurable in the same Sicherheitsbenachrichtigung UI section (`quiet_hours_from` / `quiet_hours_to`). During quiet hours: notifications suppressed, **events still captured + logged + emailed via SMTP fallback**.
- ~~**Twilio sign-up failure**~~ → moved off Twilio entirely. ntfy.sh has zero signup, no credit card, no email — works immediately. (Sipgate/Telegram/Twilio stay as future-swappable providers behind the same abstraction.)

**One thing the operator still needs to do (one-time, ~2 minutes — but only at Phase 4 testing time, not now):**
- Install the ntfy app on their phone (Play Store / App Store / F-Droid — all free). No account, no signup. The UI will display the QR to scan when we get there.

**Decision point:**
- **Read the design and say "approved"** to start Phase 1 (re-enable `video_ringbuffer` + verify).
- Or list what you want changed before I touch any code. Once you say "approved" I start Phase 1.

---

## 13. Files this design will create or modify

**Create:**
- `/home/quarero-00/Desktop/camera-ai/arrival_detection.py` (Jetson 2)
- `/etc/systemd/system/arrival-detection.service` (Jetson 2)
- `/home/quarero-00/Desktop/camera-ai/.env` (Jetson 2, gitignored, chmod 600)
- `tactical_mower/ros2_ws/src/robot_web_interface/src/robot_web_interface/api/routes/security_arrival.py`
- `tactical_mower/ros2_ws/src/robot_web_interface/src/robot_web_interface/services/telephony.py`
- `tactical_mower/scripts/retention_service.py`
- `/etc/systemd/system/retention-service.{service,timer}` (Jetson 1)
- `tactical_mower/web_app/.env` or compose `env_file` (Jetson 1, gitignored)

**Modify:**
- `tactical_mower/ros2_ws/src/system_bringup/launch/controller.launch.py` (uncomment ringbuffer block)
- `tactical_mower/ros2_ws/src/robot_web_interface/src/robot_web_interface/main.py` (init new router)
- `routen/settings/settings.yaml` (add `security_arrival` section)
- `.gitignore` (add `**/.env`, `**/secrets/`)
- `CODEBASE_MENTAL_MODEL.md` (after build: fix the stale person-detection notes; add new pipeline section)

**Untouched (load-bearing assertion):**
- `tactical_wp_follower_node.py` and everything under `control/` and `drive/` — the autonomy stack is unaffected.
- `live_detect.py` on Jetson 2.
- Existing routes / schedules / settings.

---

*Awaiting operator approval. Answer the open questions in §12 and either say "approved" (proceed with Phase 1) or list changes you want to the design.*
