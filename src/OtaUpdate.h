#pragma once

#include "Arduino.h"

// Error codes sent back to the app over the ota_control characteristic's
// ERROR frame ([0x03, code]) -- see boatlooder-app's OtaController for the
// matching Dart-side enum. Keep the two in sync; nothing enforces it
// structurally, this is the one wire contract in this codebase that isn't
// self-describing.
enum class OtaError : uint8_t {
    ALREADY_IN_PROGRESS = 0,
    INVALID_SIZE = 1,
    BEGIN_FAILED = 2,
    WRITE_FAILED = 3,
    END_FAILED = 4,
    NOT_STARTED = 5,
};

typedef std::function<void()> OnOtaAckCallback;
typedef std::function<void()> OnOtaOkCallback;
typedef std::function<void(OtaError)> OnOtaErrorCallback;

// Owns the Update.h state machine for a firmware push -- the protocol/
// business-logic layer, deliberately separate from BluetoothHandler's pure
// BLE transport, the same split Mavlink already has relative to
// BluetoothHandler. Knows nothing about BLE; main.cpp wires its callbacks
// to notifies on the ota_control characteristic.
class OtaUpdate {
public:
    OtaUpdate();

    void setOnAckCallback(OnOtaAckCallback callback);
    void setOnOkCallback(OnOtaOkCallback callback);
    void setOnErrorCallback(OnOtaErrorCallback callback);

    // Starts a new update: [size] is the total image size declared by the
    // app in its BEGIN frame. Fires the ACK callback on success (the app
    // should start streaming data), or ERROR (with a reason) otherwise --
    // most commonly ALREADY_IN_PROGRESS (a second BEGIN before an ABORT/
    // END) or BEGIN_FAILED (not enough space in the inactive OTA slot for
    // the declared size).
    void begin(uint32_t size);

    // A single DATA write's raw bytes, fed straight to Update.write() in
    // the order received -- no framing of its own needed, since
    // write-with-response already serializes delivery order on the wire
    // (see BluetoothHandler's ota_data characteristic). Silently fires
    // ERROR(NOT_STARTED) if no successful begin() is in progress, rather
    // than writing into nothing.
    void writeChunk(const uint8_t* data, size_t length);

    // Finalizes and verifies the image (Update.end() -- checks the
    // declared size and the image's own embedded checksum). Fires OK on
    // success; main.cpp is the one that actually reboots afterward, not
    // this class, so the OK notify has a chance to clear the BLE link
    // first.
    void end();

    // Safety net for a dropped connection mid-transfer, or an explicit
    // ABORT frame: discards whatever's been written so far. Must always
    // run before a later begin() can be trusted -- a half-written image
    // left in the inactive slot must never be mistaken for a complete one.
    void abort();

    bool inProgress() const { return _inProgress; }

private:
    bool _inProgress;
    uint32_t _expectedSize;
    uint32_t _written;
    OnOtaAckCallback _onAck;
    OnOtaOkCallback _onOk;
    OnOtaErrorCallback _onError;
};
