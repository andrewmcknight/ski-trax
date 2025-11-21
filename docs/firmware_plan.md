## Ski-Trax Firmware Plan

### Goals
- Modular, testable firmware targeting Heltec WiFi LoRa 32 v3.
- Clean UI that leverages TFT assets (bitmaps for compass, icons, etc.).
- Feature set split into reusable services (pairing, networking, sensors, rendering).

---

## 1. System Architecture

### Core Tasks (FreeRTOS or cooperative scheduler)
1. **UI Task**
   - Drives TFT updates and handles button events.
   - Maintains a navigation state machine (Startup → Pairing → Session Lobby → Tracking).
2. **Sensor Task**
   - Polls GPS (MAX-M10S), IMU (BNO055), BMP390, battery ADC.
   - Normalizes sensor payloads and updates a shared `SensorSnapshot`.
3. **LoRa Session Task**
   - Handles discovery, pairing, and telemetry exchange.
   - Maintains a `SessionState` model accessible via queues.
4. **Storage Task (optional future)**
   - Persist user nickname, last session ID, calibration data, etc.

### Shared Data Models
- `DeviceProfile { uuid, nickname, hwVersion }`
- `SessionDescriptor { sessionId, hostNickname, memberCount, channel, lastSeen }`
- `PeerState { uuid, nickname, lastLocation, lastUpdateMs, stats }`
- `SensorSnapshot { gpsFix, timestamp, lat/lon, heading, velocity, altitude, batteryPct, batteryIconIdx, imuCalStatus }`

Use mutex-protected structs or message queues for cross-task communication.

---

## 2. Pairing Experience

### 2.1 Startup Flow
1. **Boot Splash**
   - Display logo + firmware version while services come online.
   - Show spinner / progress bar tied to sensor init.
2. **Landing Screen**
   - Two soft buttons: `Create Session` (left) and `Join Session` (right).
   - Always-on top bar showing time/battery/sensor status even during pairing.

### 2.2 Nickname Entry
1. Prompt user after selecting host/join.
2. UI concept: on-screen keyboard (grid) navigated with buttons or a scrollable list of preset names + an "Edit" action.
3. Persist nickname in NVS for future boots.

### 2.3 Session Creation (Host)
- **UI**
  - After nickname: display “Broadcasting Session…” with a session code (short hash) and a list of connected members.
  - Accept button to “Start Tracking” once at least one joiner is connected (optional skip allowed).
- **Networking**
  - Host advertises `SESSION_BEACON` every 500 ms with:
    - Session ID, host UUID, channel (LoRa frequency/spreading factor), session capabilities.
    - Current member count and uptime.
  - On join request, host replies with `SESSION_ACCEPT` including encryption keys/nonce if using secure packets.

### 2.4 Session Joining
- **UI**
  - Show scrolling list of detected sessions with strength indicators (RSSI) and host names.
  - Selecting a session sends join request; show progress indicator while waiting for `SESSION_ACCEPT`.
- **Networking**
  - Device sends `JOIN_PROBE` broadcast while scanning for beacons (listen cycle ~3s).
  - When beacons received, populate list (expire entries after ~5 s of silence).
  - On selecting session, send `JOIN_REQUEST {nickname, uuid, capabilities}`.
  - Await `SESSION_ACCEPT` (with session parameters) or `SESSION_DENY`.

### 2.5 Behind-the-Scenes Considerations
- Use unique 64-bit session IDs derived from host UUID + uptime millis.
- Include CRC and simple reliability (ack/retry) for pairing messages.
- Maintain state machine:
  - `Idle → NicknameEntry → (Host|Join) → Beaconing/Scanning → Lobby`.
- Provide hooks for future encryption key exchange (Diffie-Hellman or pre-shared key).

---

## 3. Tracking Experience

### 3.1 Session Lobby
- Shows roster tiles (nickname, status, last fix).
- Host can start/stop session; joiners wait for start signal.
- Behind the scenes, LoRa task flips from pairing channel to tracking channel and begins periodic position broadcast.

### 3.2 Tracking Screen Layout
```
┌───────────────────────────── Top Bar (persistent) ───────────────────────────┐
│   HH:MM (GPS)     Battery Icon     Sensor Icons (GNSS, IMU, LoRa, BMP)       │
├──────────────────────────────── Content Region ─────────────────────────────┤
│                                                                             │
│   [Peer Name + status]                                                      │
│   [Compass card (full screen) | overlays]                                   │
│   [Distance Icon + value | Relative Bearing text | Altitude icon + delta]   │
│   [Last update time (“15s ago”)]                                            │
│                                                                             │
├──────────────────────────── Bottom Buttons/Indicators ───────────────────────┤
│   [< Prev Member]           [Center/Home]           [Next Member >]          │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.3 Button Mapping
- **Left/Right external buttons**: cycle through peers (wrap-around).
- **Center button**: toggles detail overlay (compass vs. map view in future).
- **Long-press combos**: e.g., hold left+right to open session menu.

### 3.4 Data Presentation
- **Compass**: use 36-direction bitmaps with overlays:
  - Add arrow/label pointing toward selected peer’s bearing relative to device heading.
  - Optionally show mini-map dot layout in future (reserve screen layer).
- **Distance/Altitude**: use custom icons (see Section 5) with values (e.g., `1.2 km`, `+80 m`).
- **Last Update**: relative time string (e.g., `Updated 12s ago`). If stale (>60s), highlight in yellow/red.
- **Peer Status**: include indicators for battery low, no fix, etc.

### 3.5 Networking (Tracking Mode)
- Each member broadcasts `TRACK_UPDATE` every X seconds (adjustable, default 2 s when moving, 5 s idle).
- Payload includes:
  - Timestamp, GPS lat/lon/alt, horizontal + vertical accuracy, heading, speed.
  - Sensor flags/battery level.
  - Sequence number for packet loss detection.
- Received updates stored in `PeerState` list; UI uses freshest data.
- Out-of-date peers flagged (greyed out in carousel).

---

## 4. Top Bar Design

### Contents
1. **GPS Time**
   - Use MAX-M10S time (UTC) converted to local offset (configurable).
   - Show `HH:MM` + small “GPS” icon if fix valid; flash or gray when no fix.
2. **Battery Icon**
   - Use one of 5 provided icons (Very Low, Low, Mid, High, Charging).
   - Battery ADC sampling handled by sensor task; smoothing filter to avoid jitter.
3. **Sensor Status Icons**
   - GNSS, IMU, BMP, LoRa, buttons/backlight.
   - Each icon has ON/OFF/Warning states (color-coded or using alternate bitmaps).

### Behavior
- Always rendered first each frame; UI task re-draws only when values change.
- Provide animation hook (e.g., blink LoRa icon when transmitting).

---

## 5. Custom Assets & Icons

### Required Assets
1. **Compass Needle Set**: 36 frames (already exist).
2. **Battery Icons**: 5 states (0%, 25%, 50%, 75%, 100% + charging overlay).
3. **Sensor Icons**:
   - GNSS lock/unlock.
   - IMU calibration status.
   - LoRa link (connected/searching).
   - BMP sensor status.
4. **Session & Menu Icons**:
   - Create session, join session buttons.
   - Nickname entry keyboard keys.
5. **Distance & Altitude Icons**:
   - Up/down arrows or mountain icon.
   - Distance circle or target icon.
6. **Misc**:
   - Status warnings (e.g., “No Fix”, “Low Battery”).
   - Soft button indicators for on-screen prompts.

### Asset Pipeline
- Store bitmaps in `code/main/src/assets/` as `const uint16_t[] PROGMEM`.
- Provide `assets_manifest.h` for centralized declarations; UI modules include via this manifest.
- Use `tools/png_to_bitmap.py` to convert new PNGs and keep aspect ratios consistent.

---

## 6. UI Navigation State Machine

```
BOOT
  └─► INIT (hardware checks, sensor init, asset preload)
        └─► LANDING (Create vs Join)
              ├─► NICKNAME_ENTRY
              │      └─► HOST_SETUP → LOBBY_HOST
              │                     └─► TRACKING
              └─► JOIN_SCAN → SESSION_LIST → LOBBY_GUEST → TRACKING
TRACKING
  └─► MENU (long-press) → SETTINGS / EXIT SESSION
```

- Each state has:
  - `enter()` for drawing static layout.
  - `update()` for periodic tasks.
  - `handleEvent(event)` for button/network callbacks.
- Allows future states (e.g., message view, map view) to be added cleanly.

---

## 7. Networking Protocol Summary

| Message           | Direction | Purpose                              | Key Fields                                        |
|-------------------|-----------|--------------------------------------|---------------------------------------------------|
| `SESSION_BEACON`  | Host→All  | Advertise session availability       | sessionId, hostName, channel, members, uptime     |
| `JOIN_PROBE`      | All→All   | Trigger hosts to respond             | uuid, nickname (optional)                         |
| `JOIN_REQUEST`    | Joiner→Host | Request to enter session           | uuid, nickname, fwVersion, capabilities           |
| `SESSION_ACCEPT`  | Host→Joiner | Confirm join + share session params | sessionId, channel, encryption seed, member list  |
| `SESSION_DENY`    | Host→Joiner | Reject join (full, auth fail)      | reason code                                       |
| `TRACK_UPDATE`    | All↔All  | Share GPS/IMU status in session      | timestamp, lat/lon/alt, heading, speed, battery   |
| `HEARTBEAT`       | All↔All  | Keepalive + version check            | uptime, metrics                                   |

- Use SX1262 raw packets via LoRa API; consider encryption/compression later.
- Each packet includes CRC and version field for compatibility.

---

## 8. Button & Input Handling

- Debounce + hold detection done centrally (button service).
- Emit high-level events: `BTN_LEFT_SHORT`, `BTN_RIGHT_LONG`, etc.
- UI states subscribe to events to change screens or trigger actions.
- Allows future expansion (e.g., double-tap for quick actions).

---

## 9. Settings & Future Proofing

- **Settings Menu** (accessible via long-press):
  - Units (metric/imperial).
  - Display brightness/backlight timeout.
  - Radio channel/power overrides.
  - Sensor calibration helper screens.
- **Modularity**:
  - Keep `ui/`, `sensors/`, `radio/`, `assets/`, `config/` directories.
  - Use interfaces (pure virtual classes or function pointers) to abstract hardware.
  - Provide test harness in `code/io_tests/` for each subsystem (e.g., pairing mock, UI emulator).

---

## 10. Constraints & Open Questions
- **Security**: No PIN/password protection for initial version.
- **Networking**: Point-to-point session only (no mesh); optimize for two devices but design protocol for up to 8 members max.
- **User Count**: Hard cap 8; UI should scale but initial validation can be done with 2 devices.
- **Units**: Imperial (feet/mph) by default; metric support can be added later.
- **Feature Scope**: Prioritize polished UX + core functionality over production-grade extras (logging, encryption, etc.).

Remaining questions:
1. Should joiners store last session info for auto-reconnect?
2. Any requirements for exporting activity logs/tracks later?
3. How should we visualize “inactive” peers (e.g., offline indicator)?
