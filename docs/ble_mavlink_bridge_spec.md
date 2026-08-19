# Firmware spec: BLE MAVLink telemetry + param bridge

Implementation spec for the ESP32 side of the design in the app repo's
`docs/mavlink_ble_bridge_concept.md`. That doc covers both sides at a
concept level; this one is the concrete firmware punch list — exact
files, functions, and message routing.

## Does `Mavlink.h` need to be re-created?

**No.** The parser is already transport-agnostic and already correct:
`handleReceivedByte()` feeds `mavlink_parse_char()` one UART byte at a
time and gets a validated `mavlink_message_t` in `_msg` on every
complete frame. That machinery doesn't change. What's actually missing
is three additive things:

1. `setupStreamingRates()` currently **disables** almost every message
   this needs (see its `disableMessages[]` list) instead of requesting
   them — a config change, not a structural one.
2. Received messages outside `SERVO_OUTPUT_RAW`/`HEARTBEAT` are parsed
   into `_msg` and then dropped on the floor — nothing reads them.
3. There's no path from "UART received a message" to "BLE sent
   something", and no second byte stream for BLE-writes-going-to-UART
   (param requests) at all.

So: extend `Mavlink` and `BluetoothHandler`, don't replace them.

## 1. New GATT characteristics

Same service (`4fafc201-1fb5-459e-8fcc-c5c9c331914b`), two new
characteristics alongside the existing control one. UUIDs below are
freshly generated (`uuidgen`) — swap if you already reserved others:

| Characteristic | UUID | Properties | Descriptor |
|---|---|---|---|
| Control *(existing, unchanged)* | `beb5483e-36e1-4688-b7f5-ea07361b26a8` | `WRITE` | — |
| **Telemetry** | `430f885a-4c7b-40c6-bdfc-280a526fd118` | `NOTIFY` | `BLE2902` |
| **Settings** | `5cf3acbf-4809-453a-93ab-4359429056e3` | `WRITE` + `INDICATE` | `BLE2902` |

`BLE2902` is required on both new characteristics — it's the CCCD the
client writes to enable notify/indicate; without it `notify()`/
`indicate()` calls are silent no-ops against most clients.

## 2. `BluetoothHandler.h` / `.cpp` changes

**Header** — add:

```cpp
typedef std::function<void(const uint8_t* data, size_t length)> OnSettingsWriteCallback;

class BluetoothHandler {
public:
    // existing members unchanged, plus:
    void setOnSettingsWriteCallback(OnSettingsWriteCallback callback);
    void notifyTelemetry(const uint8_t* data, size_t length);
    void indicateSettings(const uint8_t* data, size_t length);

private:
    BLECharacteristic* _telemetryChar;
    BLECharacteristic* _settingsChar;
    OnSettingsWriteCallback _onSettingsWriteCallback;
};
```

**`.cpp`** — in `init()`, before `BLEDevice::init(DEVICE_NAME)`:

```cpp
BLEDevice::setMTU(247);
```

This is a *request* — actual negotiated MTU is the min of both sides'
asks and the app must also call `requestMtu()` after connecting (see
the app-side doc). Don't assume 247 lands; chunk defensively (§5).

After creating the existing control characteristic, add:

```cpp
_telemetryChar = pService->createCharacteristic(
    TELEMETRY_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
);
_telemetryChar->addDescriptor(new BLE2902());

_settingsChar = pService->createCharacteristic(
    SETTINGS_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_INDICATE
);
_settingsChar->addDescriptor(new BLE2902());
_settingsChar->setCallbacks(new SettingsCallbacks(this)); // separate
                                                            // BLECharacteristicCallbacks
                                                            // subclass, mirrors
                                                            // BoatLuderCallbacks but
                                                            // calls _onSettingsWriteCallback
```

`notifyTelemetry`/`indicateSettings`: `setValue()` + `notify()`/
`indicate()` respectively. Both need the MTU-aware chunking from §5 —
don't call `setValue()` with more bytes than the connection can carry
in one PDU.

**Open item**: confirm what this specific vendored BLE library version
exposes for reading the *negotiated* per-connection MTU
(`BLEServer::getPeerMTU(conn_id)` exists in some versions, not others).
If unavailable, chunk conservatively at a fixed size (e.g. 180 bytes)
rather than assuming the full requested 247.

## 3. `Mavlink.h` / `.cpp` changes

**Header** — add:

```cpp
enum class RelayChannel { NONE, TELEMETRY, SETTINGS_ACK };

typedef std::function<void(const uint8_t* data, size_t length)> OnRelayCallback;

class Mavlink {
public:
    // existing members unchanged, plus:
    void setOnTelemetryRelayCallback(OnRelayCallback callback);
    void setOnSettingsAckRelayCallback(OnRelayCallback callback);
    void handleBleSettingsByte(uint8_t byte);  // BLE -> UART direction

private:
    RelayChannel classifyRelay(uint16_t msgid);
    bool isAllowedFromApp(uint16_t msgid);      // safety allowlist, see §4

    // second, independent parser state — BLE and UART are unrelated
    // byte streams and must not share _status/_msg
    mavlink_message_t _bleMsg;
    mavlink_status_t _bleStatus;

    OnRelayCallback _onTelemetryRelayCallback;
    OnRelayCallback _onSettingsAckRelayCallback;
};
```

**`.cpp`** — `setupStreamingRates()`: replace the disable-list with
explicit enables per the rate table below (same
`requestMessageInterval()` call already used for `SERVO_OUTPUT_RAW`):

| Message | Rate |
|---|---|
| `HEARTBEAT` | 1 Hz *(unchanged)* |
| `VFR_HUD` | 2 Hz |
| `GPS_RAW_INT` | 1 Hz |
| `SYS_STATUS` | 1 Hz |
| `EKF_STATUS_REPORT` | 1 Hz |
| `VIBRATION` | 0.5 Hz |
| `GLOBAL_POSITION_INT` | 1 Hz |
| `HOME_POSITION` | 0.2 Hz (simplest for v1 — every 5s; an event-triggered refresh on reconnect/RTL-entry is a later optimization, not required now) |

`PARAM_VALUE` isn't in this table — it's response-driven (arrives
because something requested it), not a periodic stream.

`classifyRelay()`:

```cpp
Mavlink::RelayChannel Mavlink::classifyRelay(uint16_t msgid) {
    switch (msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
        case MAVLINK_MSG_ID_VFR_HUD:
        case MAVLINK_MSG_ID_GPS_RAW_INT:
        case MAVLINK_MSG_ID_SYS_STATUS:
        case MAVLINK_MSG_ID_EKF_STATUS_REPORT:
        case MAVLINK_MSG_ID_VIBRATION:
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        case MAVLINK_MSG_ID_HOME_POSITION:
            return RelayChannel::TELEMETRY;
        case MAVLINK_MSG_ID_PARAM_VALUE:
            return RelayChannel::SETTINGS_ACK;
        default:
            return RelayChannel::NONE;
    }
}
```

`handleReceivedByte()` — after the existing `SERVO_OUTPUT_RAW`/
`HEARTBEAT` handling, add the relay dispatch:

```cpp
RelayChannel channel = classifyRelay(_msg.msgid);
if (channel != RelayChannel::NONE) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &_msg);
    if (channel == RelayChannel::TELEMETRY && _onTelemetryRelayCallback) {
        _onTelemetryRelayCallback(buf, len);
    } else if (channel == RelayChannel::SETTINGS_ACK && _onSettingsAckRelayCallback) {
        _onSettingsAckRelayCallback(buf, len);
    }
}
```

Note this re-serializes the already-validated `_msg` rather than
decoding it into a typed struct — no per-message-type decode code is
needed on the firmware side at all, relay is purely msgid-driven. If a
later feature needs the firmware itself to *read* a value (not just
forward it), decode that one message type then, not preemptively.

`handleBleSettingsByte()` — new, mirrors `handleReceivedByte()` but
writes to UART instead of relaying to BLE, and enforces the allowlist
from §4:

```cpp
void Mavlink::handleBleSettingsByte(uint8_t byte) {
    if (mavlink_parse_char(MAVLINK_COMM_1, byte, &_bleMsg, &_bleStatus)) {
        if (!isAllowedFromApp(_bleMsg.msgid)) {
            LOG_WARNF("Dropped disallowed msgid %d from BLE settings channel\n", _bleMsg.msgid);
            return;
        }
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &_bleMsg);
        _mavSerial.write(buf, len);
    }
}
```

Use `MAVLINK_COMM_1` (a distinct channel constant from the UART
parser's `MAVLINK_COMM_0`) — the mavlink C library keys parser state by
channel, so reusing `MAVLINK_COMM_0` for both streams would corrupt
both parsers' internal buffers.

## 4. Safety allowlist on the BLE→UART settings path — don't skip this

`handleBleSettingsByte()` forwards *validated* MAVLink to the flight
controller verbatim. Without a check on `msgid`, that channel could
relay *any* MAVLink command the app (buggy or malicious) chooses to
send — including things like `COMMAND_LONG` with
`MAV_CMD_COMPONENT_ARM_DISARM` or `MAV_CMD_DO_SET_MODE`, which would
bypass the boat's actual arm/mode control path entirely (the
RC-override scheme on the Control characteristic). That's a real safety
gap on a vehicle that can arm real motors, not a hypothetical one.

`isAllowedFromApp()` should be a tight allowlist, not a denylist:

```cpp
bool Mavlink::isAllowedFromApp(uint16_t msgid) {
    switch (msgid) {
        case MAVLINK_MSG_ID_PARAM_SET:
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            return true;
        default:
            return false;
    }
}
```

Everything else arriving on the Settings write path gets logged and
dropped. This is the one place in the whole design where firmware
*does* need to know MAVLink semantics, specifically because it's a
security boundary, not just a relay.

## 5. Chunking / MTU handling

Both `notifyTelemetry()` and `indicateSettings()` need to split payloads
larger than the connection's usable PDU size
(negotiated MTU − 3 bytes ATT overhead). At MTU 247 (≈244 usable) this
whitelist's messages (9–43 bytes) essentially never fragment; plan for
it anyway since negotiated MTU varies by client OS and isn't
guaranteed to reach 247:

```cpp
void BluetoothHandler::notifyTelemetry(const uint8_t* data, size_t length) {
    const size_t chunkSize = 180; // conservative; see MTU note in §2
    for (size_t offset = 0; offset < length; offset += chunkSize) {
        size_t n = min(chunkSize, length - offset);
        _telemetryChar->setValue((uint8_t*)(data + offset), n);
        _telemetryChar->notify();
        if (offset + n < length) {
            delay(3); // pace back-to-back notifies — see §7
        }
    }
}
```

Since this reuses the same byte-stream parser design as UART, a lost or
truncated chunk just fails that one message's checksum on the app side
and gets discarded — it doesn't desync anything for the next message
(see the app-side doc's reliability discussion). `indicateSettings()`
is the same shape but calls `indicate()`.

## 6. `.ino` wiring

New prototype + implementation, same shape as the existing
`onBluetoothWrite`:

```cpp
void onBluetoothSettingsWrite(const uint8_t* data, size_t length) {
    for (size_t i = 0; i < length; i++) {
        mavlink.handleBleSettingsByte(data[i]);
    }
}
```

In `setup()`, alongside the existing three callback registrations:

```cpp
btHandler.setOnSettingsWriteCallback(onBluetoothSettingsWrite);
mavlink.setOnTelemetryRelayCallback([](const uint8_t* data, size_t len) {
    btHandler.notifyTelemetry(data, len);
});
mavlink.setOnSettingsAckRelayCallback([](const uint8_t* data, size_t len) {
    btHandler.indicateSettings(data, len);
});
```

(Captureless lambdas referencing the global `btHandler` convert cleanly
to the existing `std::function`-based callback types — same pattern
already used for `onBluetoothConnect`/`onBluetoothDisconnect`.)

## 7. Things to verify while implementing, not assume

- **Notify/indicate pacing.** The ESP32 Arduino BLE stack can silently
  drop a `notify()`/`indicate()` call made while a previous one for the
  same characteristic is still in flight. The `delay(3)` in §5 is a
  guess, not a measurement — watch actual drop behavior under the full
  ~7.5 msg/sec telemetry load and tune (or move to a small outbound
  queue + task if a fixed delay isn't enough).
- **Negotiated MTU query API** — confirm what this exact vendored BLE
  library version exposes (§2's open item) before hardcoding a chunk
  size.
- **`HOME_POSITION` staleness** — the 0.2 Hz periodic poll (§3) means
  up to 5s of stale distance-to-home right after a home reset (e.g.
  re-arming somewhere new). Fine for v1; revisit if that's noticeable
  in practice.

## 8. File-by-file summary

| File | Change |
|---|---|
| `Mavlink.h` | Extend: `RelayChannel` enum, two new callback types + setters, second parser state (`_bleMsg`/`_bleStatus`), `handleBleSettingsByte()`, `classifyRelay()`, `isAllowedFromApp()` declarations |
| `Mavlink.cpp` | Extend: rewrite `setupStreamingRates()` to enable the §3 whitelist, add relay dispatch in `handleReceivedByte()`, implement `handleBleSettingsByte()`, `classifyRelay()`, `isAllowedFromApp()` |
| `BluetoothHandler.h` | Extend: two new `BLECharacteristic*` members, `OnSettingsWriteCallback` type, `notifyTelemetry()`/`indicateSettings()`/`setOnSettingsWriteCallback()` declarations |
| `BluetoothHandler.cpp` | Extend: `BLEDevice::setMTU(247)`, create both new characteristics + `BLE2902` descriptors, `SettingsCallbacks` class, implement the three new methods |
| `boatlooder-esp32.ino` | Extend: `onBluetoothSettingsWrite()`, three new callback registrations in `setup()` |

No file needs a rewrite from scratch — every change above is additive
to the existing structure.
