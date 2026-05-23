# PLAN — UI customer hand-off (2-day sprint)

**Status:** PLAN — APPROVED, NOT STARTED. Operator approved this scope and ordering on 2026-05-23.

**Pickup rule for a future Claude session:**
1. Read `CODEBASE_MENTAL_MODEL.md` first (auto-loaded via the SessionStart hook).
2. Read this file end-to-end.
3. Read the "What is done / what is pending" table in §6 to find the exact next item.
4. Confirm with the operator before resuming — the customer hand-off date and any new constraints may have shifted.

Companion docs:
- `CODEBASE_MENTAL_MODEL.md` — full architecture (what every file does, every safety scar).
- `DESIGN_ARRIVAL_PIPELINE.md` — the now-shipped arrival-detection pipeline (Phase 1–6).
- `CLAUDE.md` — operating rules; this work must stay on a branch, one commit per item, never `git add -A`.

---

## 1. Why this exists

The current web UI was built for a robotics-literate operator (jargon in the top nav: GNSS1/GNSS2/IMU/Fusion; technical terms throughout: RTK, waypoint_tolerance, speed_factor, charge_point vs home_point, ntfy topic). The customer who receives this robot is a **non-technical older user**. Without UI changes the customer will need a full operator to use it — defeating the unattended-security premise.

We had a recent live example of the friction: in Phase 6 the operator set the quiet-hours "from" field but didn't realize they also had to set the "bis" field; the UI saved the half-set state silently, the dispatcher (correctly) treated it as "no quiet window", and they got rung during what they thought was their quiet hour. A non-technical user would never recover from that confusion.

## 2. Goals

A non-technical 60+ user can, **without training**:
- See at a glance whether the robot is OK (one indicator, plain German).
- Start a scheduled patrol.
- Receive an alarm on their phone with one-tap acknowledge.
- Watch a captured clip.
- Change their phone number / quiet hours.

## 3. Constraints

- **2 working days.** That's ~16 hours of dev time.
- **No risk to the autonomy stack.** Only `tactical_mower/static/` and `tactical_mower/ros2_ws/src/robot_web_interface/` are in scope. `control/`, `drive/`, `system_bringup/`, `eneo_event_publisher/`, `video_ringbuffer/` — DO NOT TOUCH.
- **No DB changes.** Settings already live in `routen/settings/settings.yaml`. Add fields, don't migrate.
- **Native German UI throughout.** Operator language is German; never mix English/German in user-facing strings (CLAUDE.md MED-5).
- **All changes on a dedicated branch.** `ui-customer-handoff`. One commit per item from the table below.
- **CSS-first where possible.** Heavy DOM restructuring is risky in 2 days; rebrand + reorganize first, then surgical structure changes.

## 4. Operator decisions made

| Item | Decision | Reason |
|---|---|---|
| #3 Persistent STOPP button | **DROPPED** (operator decision 2026-05-23) | Operator deemed not important right now. *If a future session adds it back, the spec is still in §7.* |
| #6 Route-aufnehmen mode (drive manually → save trail) | **DROPPED** | 1-2 day project alone; doesn't fit the 2-day window. |
| QR-code lib for #5 | Vendor a small JS lib (~5 KB), e.g. `qrcode-svg`, **client-side**. No Python `qrcode` pip dep — that would force a Docker image rebuild. |
| Customer device | **CONFIRMED 2026-05-23: laptop AND tablet** — design for both, 1280×720 and 768×1024. Touch targets must still be ≥ 48px (tablet-friendly). Test at both viewports after every commit. |
| Operator on-site at hand-off? | **CONFIRMED 2026-05-23: yes, operator present on day 1** — therefore wizard #9 is **lower priority**. If D2 runs long, ship the dashboard-checklist fallback (§7 #9) instead of the full modal wizard. Operator can verbally walk the customer through setup. Full wizard still has value for later (different operator, new sites) — ship if time. |

## 5. The 2-day plan (the agreed scope)

### Day 1 — foundation + biggest friction point

#### D1 morning — 4 h
- **#8 Accessibility base CSS pass** (~30 min) — bulk-applied across all pages
- **#2 Single status badge** (~1.5 h) — replaces GNSS1/GNSS2/IMU/Fusion in nav
- **#1 Dashboard simplification** (~2 h) — one big status card + 3 big action buttons

#### D1 afternoon — 4 h
- **#5 Notifications cleanup + QR + validation** (~3-4 h) — addresses the bug class that already bit us

### Day 2 — content + onboarding + polish

#### D2 morning — 4 h
- **#4 Settings basic/advanced split** (~3 h)
- **#7 Event thumbnails** (~1 h, quick because frame.jpg is already saved per event)

#### D2 afternoon — 4 h
- **#9 First-launch wizard** (~3 h)
- **Browser test pass + dry-run + fix-as-you-find** (~1 h)

## 6. What is done / what is pending

Update this table after each commit. Future Claude reads it to find the next item.

| # | Item | Status | Commit SHA | Notes |
|---|---|---|---|---|
| #8 | Accessibility base CSS | TODO | — | bulk CSS in `common.css` |
| #2 | Single status badge | TODO | — | replaces 4 badges in nav |
| #1 | Dashboard simplification | TODO | — | `index.html` + `index.js` |
| #5 | Notifications cleanup | TODO | — | `settings.html` + new endpoints if needed |
| #4 | Settings split | TODO | — | `settings.html` reorg, no logic change |
| #7 | Event thumbnails | TODO | — | `events.html` |
| #9 | First-launch wizard | TODO | — | new component on `index.html` |

## 7. Per-item technical spec

### #8 — Accessibility base CSS pass
**Files**: `tactical_mower/static/css/common.css` (and possibly `settings.css`).
**Approach**: variables at the top of `common.css`:
```css
:root {
  --font-base: 18px;        /* was ~14 */
  --font-large: 22px;
  --btn-min: 48px;          /* Apple HIG / Material */
  --focus-ring: 3px solid #0066cc;
  --color-text: #1a1a1a;    /* WCAG AA ≥ 7:1 on white */
  --color-text-muted: #4a4a4a;  /* ≥ 4.5:1 */
}
html { font-size: var(--font-base); }
button, input, select, textarea { font-size: 1rem; min-height: var(--btn-min); }
*:focus-visible { outline: var(--focus-ring); outline-offset: 2px; }
```
**Acceptance**: every page renders with body text ≥ 18px, every button has tap area ≥ 48×48, keyboard tab shows visible focus ring.
**Risk**: low; pure CSS, no JS, no schema change.

### #2 — Single status badge
**Files**: `tactical_mower/static/index.html`, `tactical_mower/static/css/common.css`, `tactical_mower/static/js/index.js` (and whatever wires the nav badges today).
**Approach**:
- Read the 4 fusion fields from the existing `/ws/robot_state` WebSocket (already published).
- Compute one of three states:
  - 🟢 **"Alles bereit"** — `gnss1_status == 8 && gnss2_status == 8 && fusion_init_status == 2`
  - 🟡 **"GPS schwach"** — both GNSS ≥ 5 (RTK_FLOAT) but not 8
  - 🔴 **"GPS verloren"** — anything worse, OR fusion not init, OR stale data >5 s
- Render as one big dot in the nav (replaces the 4 small badges). Tap → expand modal showing the four old badges for the curious / a technician.
**Acceptance**: badge updates within 2 s of underlying RTK status change; tapping shows the detail modal; operator never sees raw status codes on the main page.
**Risk**: medium — must hook into the existing WebSocket; verify by force-killing RTK and confirming the badge goes red within 5 s.

### #1 — Dashboard simplification (NOT a full rewrite)
**Files**: `tactical_mower/static/index.html`, `tactical_mower/static/js/index.js`.
**Approach**: insert ONE big card at the top of `.container` that occupies ~30 % of viewport height. It shows:
- The state from `/tactical/robot/state` mapped to plain German:
  - `UNINITIALIZED` → "Roboter startet — bitte warten"
  - `MANUAL` → "Bereit, wartet auf Anweisung"
  - `IDLE` → "Bereit, wartet auf Anweisung"
  - `DOCKED` / `CHARGING` → "Lädt — {battery_pct} %"
  - `NAVIGATING` → "Fährt Route '{active_route.name}' (WP {current_wp_idx}/{total})"
  - `RETURNING_TO_HOME` → "Auf dem Weg zur Ladestation"
  - `DOCKING` / `UNDOCKING` → "Bewegt sich zur Ladestation"
  - `ERROR` → "Problem — bitte Service rufen"
- Battery as a horizontal bar with German labels (e.g. "Akku 73 % — ungefähr 4 Std. Laufzeit verbleibend").
- Three big buttons under it (50px+ tall): **[Route starten]** **[Roboter anhalten]** **[Manuell fahren]**.
- Existing map shrinks to ~30 % below this card.
- Existing technical readouts go into a `<details>` accordion: "Details anzeigen".
**Acceptance**: a non-technical user can answer "is the robot OK?" within 2 seconds of opening the page, without any jargon.
**Risk**: medium — the existing dashboard is complex; aim for *additive* (insert new card on top, hide most old content under accordion) rather than *rewrite*.

### #5 — Notifications cleanup + QR + validation
**Files**:
- `tactical_mower/static/settings.html` (the Sicherheitsbenachrichtigung panel + the inline `<script>` we added in DESIGN_ARRIVAL_PIPELINE.md §5.7)
- `tactical_mower/static/js/qrcode.min.js` — NEW, vendored JS QR lib (try [qrcode-generator](https://github.com/kazuhikoarase/qrcode-generator) or [davidshimjs/qrcodejs](https://github.com/davidshimjs/qrcodejs) — both MIT, ~5 KB, drop-in)
- `tactical_mower/ros2_ws/src/robot_web_interface/src/robot_web_interface/api/routes/settings.py` (validation tightening)

**Changes**:
1. **Rename labels** everywhere:
   - Section heading: "Sicherheitsbenachrichtigung (Ankunft)" → **"Handy-Alarm"**
   - "ntfy Topic" → hide entirely behind a collapsed `<details>` element labeled "Erweitert"
   - "Test-Benachrichtigung senden" → **"Jetzt testen — Handy klingelt"**
2. **QR code rendering**: replace the text URL with a 200×200 QR + below it the URL as a small monospace tap-to-copy line.
3. **Form validation** (both client- and server-side):
   - If `quiet_hours_from` is set, `quiet_hours_to` is **required** (and vice versa). Save button disabled with German error "Bitte beide Zeitfelder ausfüllen oder beide leer lassen".
   - If `notifications.enabled = true` and topic empty, server already auto-generates — keep that, surface a banner "Neues Topic wurde automatisch erzeugt — bitte QR scannen".
4. **Connectivity hint**: persist `last_test_at` and `last_test_success` in settings.yaml; render as "Letzte Test-Benachrichtigung: 14:32 ✓" / "Noch nie getestet — drücken Sie Jetzt testen".
5. **Honor quiet hours in the test endpoint** (decided to flip from earlier): so the test actually simulates what an arrival would do. If in quiet window, the test button shows "In Ruhezeit — Test-Benachrichtigung würde unterdrückt. Trotzdem senden?" with a one-click override.

**Acceptance**: setting only one of the quiet hours fields is impossible (UI blocks save). QR scannable from operator's phone in <10 s. Test button shows quiet-hour state.
**Risk**: low — mostly relabeling + a JS lib drop + one validation rule.

### #4 — Settings basic/advanced split
**Files**: `tactical_mower/static/settings.html`, `tactical_mower/static/css/settings.css`.
**Approach**: wrap the existing 7 panels in 2 tabs:
- **Grundeinstellungen** (default open):
  1. Ladestation & Home-Position (rename to **"Ladestation"** — operator doesn't need to know about home_point distinction)
  2. Akku-Mindestladung
  3. Handy-Alarm (the panel from #5)
  4. E-Mail-Benachrichtigungen (just the enabled toggle + recipients line; everything else SMTP-config goes to Advanced)
  5. Daten automatisch löschen (one toggle: "Aufnahmen nach 72 Stunden löschen — Datenschutz" — wraps `security_arrival.retention_hours`)
- **Erweitert** (collapsed):
  - Existing Einstellungen panel (speed_factor, max_route_distance, waypoint_tolerance, enable_obstacle_avoidance)
  - Full SMTP config (host/port/use_tls/user/password/sender)
  - ntfy topic + rotate
  - Aufnahmefenster (pre/post seconds)
  - Restore settings panel

**Inline plain-language explainers** under every Grund field (max 12 German words each). Example for Akku-Mindestladung: *"Bei diesem Wert fährt der Roboter zur Ladestation zurück."*

**Acceptance**: page on first load shows only Grundeinstellungen with ≤ 6 cards visible. Erweitert expands cleanly. Operator can complete daily tasks without ever opening Erweitert.
**Risk**: medium — restructures all panels; do the tab wrapper first, prove it works, then iteratively move panels.

### #7 — Event thumbnails
**Files**: `tactical_mower/static/events.html`, `tactical_mower/static/css/events.css` (likely new), `tactical_mower/static/js/events.js` (or inline).
**Approach**:
- Backend already serves `GET /api/events` returning index entries, and there's a `frame.jpg` per event in the directory. Add a new endpoint `GET /api/events/{event_id}/frame` that serves the JPG (FileResponse, same pattern as the video endpoint in `events.py`). Or reuse the existing video route with `?type=frame` — pick whichever is less invasive.
- Render a CSS grid of cards (3 cols on tablet, 2 on phone). Each card:
  - 16:9 thumbnail of `frame.jpg`
  - Big German label: **"🚶 Person — heute 14:43 Uhr"** (class_label emoji + relative time)
  - Tap card → open clip in a fullscreen video player overlay
  - Long-press → delete with confirmation
- Filter chips at the top: **Heute · Gestern · Letzte 7 Tage · Alle**. No date pickers.

**Acceptance**: tap a thumbnail → video plays inline within 2 s. Visual scanning of last 24 h of events takes ≤ 5 s.
**Risk**: low-medium — depends on adding the frame-serving endpoint cleanly.

### #9 — First-launch wizard
**Files**: `tactical_mower/static/index.html` (modal HTML), `tactical_mower/static/js/wizard.js` (new), `tactical_mower/static/css/wizard.css` (new).
**Approach**:
- On page load, fetch `/api/settings/home-position` + `/api/settings/notifications` + `/api/settings/advanced`.
- If `charge_point.latitude == null` OR `security_arrival.notifications.enabled == false`, show a full-screen modal.
- 4 steps:
  1. **"Willkommen"** — one-line welcome + "Wir richten in 2 Minuten alles ein."
  2. **"Wo lädt der Roboter?"** — wait for GPS lock (display "GPS sucht…" with a spinner); when fix is good, show "Roboter steht hier. Als Ladestation speichern?" → one button → POST `/api/settings/home-position`.
  3. **"Wo soll der Alarm hin?"** — show QR (reusing #5's QR component). Step says "Installieren Sie die ntfy-App, scannen Sie den QR, drücken Sie unten 'Test'". The test button calls `/api/security/notifications/test`; UI waits for the operator to confirm "Handy hat geklingelt" → next.
  4. **"Wann nicht stören?"** — two time pickers with smart default 22:00 → 07:00. "Standard übernehmen" preset. Save → close modal.
- After modal closes, land on the simplified dashboard.
- Modal is skippable but re-appears next page-load if mandatory fields still empty.

**Acceptance**: an operator who has never used the system can complete first-time setup in ≤ 3 min without external help.
**Risk**: medium-high — biggest unknown of the sprint. Has a fallback: if running over time on D2, ship a simpler "Setup-Checkliste" panel on the dashboard (a list of "✗ Ladestation noch nicht gesetzt — jetzt einrichten" rows that link to settings).

## 7b. #3 (STOPP button) — preserved spec for future re-enablement

(Dropped per operator 2026-05-23 but worth keeping the spec since it's small and the operator may change their mind.)

**Files**: `tactical_mower/static/css/common.css`, `tactical_mower/static/js/common.js`, EVERY HTML page's `<nav>`.
**Approach**:
- Big red button fixed top-right of nav, always visible: **`■ STOPP`**.
- Tap → fetch POST `/api/control/emergency_stop` with `{state: true}` → motors zero.
- Full-screen overlay appears: "Roboter angehalten. Knopf hier erneut drücken zum Fortsetzen." with one button.
- Re-tap → POST `{state: false}` → overlay closes.
- No "are you sure" prompt — emergency stops must not have confirmation friction.
- Style: 60×60 px square, color `#cc0000`, white square icon, bold drop shadow. Should be impossible to miss.

**Backend already provides** the service: `interfaces/srv/CommandControl` on topic `/control/emergency_stop`. Implementation is ~1 hour of work.

## 8. Operational rules during the sprint

- Branch: `ui-customer-handoff`
- One commit per item from the table above
- Push to origin AND nx-orin after every commit (we have working PAT)
- Update the `Status` and `Commit SHA` columns in §6 after every commit
- Before any change to `static/*.html`, **make a copy** of the original next to it (e.g. `index.html.bak`) until the new version is operator-approved. Revert via `mv index.html.bak index.html`.
- **No `docker compose build`** during the sprint unless requirements change — `static/` is bind-mounted? **CHECK FIRST**: per the AUDIT.md note and the Phase 3 finding, static IS NOT bind-mounted on web_app; it's copied at image-build time. So edits to `static/*.html` require either `docker compose build web_app` (slow, ~2 min) OR a temporary bind-mount override during the sprint. **Recommend**: add `- ./static:/app/static` to `docker-compose.override.yaml` web_app service for the duration of the sprint, then revert before customer hand-off.
- Test in browser at **1280×720** AND **768×1024** after every change.
- After each commit: ask operator "look right?" before moving on. They are the proxy for the customer.

## 9. Definition of done (for the customer hand-off)

The 2-day sprint is "done" when:
- [ ] All 7 in-scope items have status `DONE` in §6 (or wizard has been swapped for the dashboard-checklist fallback per §9 of item #9 spec)
- [ ] Final commit on `ui-customer-handoff` pushed to both remotes
- [ ] Operator has used every page on the target device size and tapped every primary button
- [ ] Fresh-install dry-run completed: delete `charge_point` + `notifications.enabled=false` in settings.yaml, reload, walk through the wizard end-to-end. Phone rings on test push. Walk in front of camera → event appears in Ereignisse with thumbnail → video plays inline.
- [ ] The `static/*.html.bak` files have been deleted (after final operator approval)
- [ ] `docker-compose.override.yaml` reverted to remove the static bind-mount (so the image is canonical again)
- [ ] One final commit: "ui: customer hand-off ready — sprint summary" with a CHANGELOG entry listing what changed

## 10. Open questions blocking start

**All resolved 2026-05-23.** See §4 for the answers — recorded there as the authoritative spec.

Summary:
1. ✅ Customer uses **laptop AND tablet**. Build responsive for 1280×720 + 768×1024.
2. ✅ Operator **will be present on day 1** of hand-off. Wizard #9 → lower priority; checklist fallback is acceptable if time runs short.

**Sprint is unblocked. Next step: operator says "go" → branch `ui-customer-handoff` → start item #8.**

## 11. Pickup checklist for next Claude session

If you (future Claude) are reading this for the first time and need to continue:

1. Read `CODEBASE_MENTAL_MODEL.md` (auto-loaded via SessionStart hook — if not, Read it manually).
2. Read this file end-to-end.
3. `git checkout ui-customer-handoff` (or create the branch if it doesn't exist yet).
4. Open §6 table — find the first item with status `TODO`. That's where to start.
5. Read the matching per-item spec in §7.
6. Confirm with operator before making changes (CLAUDE.md §1 / §9).
7. After your commit, update §6 with status `DONE` and the commit SHA.
8. Push to both remotes.

---

*Saved 2026-05-23. Next: operator approval to branch + start with #8 (accessibility CSS pass) followed by #2 (single status badge). All items in §6 are pending.*
