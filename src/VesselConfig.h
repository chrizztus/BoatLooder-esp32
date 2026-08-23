#pragma once

#include "Arduino.h"
#include <Preferences.h>
#include "motor_drivers/MotorDriverBackend.h"

// Compile-time fallback, used only until a name is persisted -- see
// currentName(). App's BleController.deviceNamePrefix must match
// APP_FILTER_PREFIX exactly, including the "@" it prepends -- keep both
// in sync by hand, there's no shared source of truth across repos.
#define APP_FILTER_PREFIX "boatlooder"
#define DEFAULT_VESSEL_NAME "boatlooder"

// Longest name SET_NAME will accept -- matches DEFAULT_VESSEL_NAME's own
// length, conservative against BLE's legacy 31-byte advertising payload
// budget once the "boatlooder@" prefix and everything else in the
// advertisement is accounted for.
#define VESSEL_NAME_MAX_LEN 20

// Error codes sent back to the app over vessel_config's ERROR frame
// ([0x03, code]) -- see boatlooder-app's VesselConfigController for the
// matching Dart-side enum. Keep the two in sync; nothing enforces it
// structurally, same caveat OtaUpdate.h's OtaError carries.
enum class VesselConfigError : uint8_t {
    NAME_TOO_LONG = 0,
    ARMED = 1,
    INVALID_DRIVER = 2,
};

typedef std::function<void()> OnNameOkCallback;
typedef std::function<void()> OnDriverOkCallback;
typedef std::function<void(VesselConfigError)> OnVesselConfigErrorCallback;

// Owns the vessel's persisted identity (name, motor driver choice) in
// NVS -- the protocol/business-logic layer, the same split from
// BluetoothHandler's pure BLE transport that OtaUpdate already has.
// Knows nothing about BLE or Mavlink directly (vehicleArmed is passed
// into setMotorDriver() by main.cpp, not read from here); main.cpp wires
// this class's callbacks to notifies on the vessel_config characteristic.
class VesselConfig {
public:
    VesselConfig();

    void setOnNameOkCallback(OnNameOkCallback callback);
    void setOnDriverOkCallback(OnDriverOkCallback callback);
    void setOnErrorCallback(OnVesselConfigErrorCallback callback);

    // Opens NVS and loads whatever's persisted, falling back to
    // DEFAULT_VESSEL_NAME / MotorDriverType::Bts7960 if nothing's stored
    // yet. Call once at setup(), before anything reads currentName()/
    // currentMotorDriver().
    void init();

    // The bare, editable vessel name -- never includes APP_FILTER_PREFIX.
    // main.cpp composes the full advertised name
    // (APP_FILTER_PREFIX "@" + currentName()) once at boot.
    String currentName() const { return _name; }
    MotorDriverType currentMotorDriver() const { return _driver; }

    // Validates length, persists, fires NAME_OK/ERROR(NAME_TOO_LONG). No
    // live effect -- BluetoothHandler::init() only reads currentName() at
    // boot, so this only takes effect on the next reboot.
    void setName(const uint8_t* data, size_t length);

    // vehicleArmed is the real safety boundary here, not just an
    // app-side disabled control -- rejects with ERROR(ARMED) if true.
    // Validates driverId, persists, fires DRIVER_OK. Does not touch any
    // pins itself: main.cpp's thrustControlTask polls
    // currentMotorDriver() each loop and applies a change through
    // MotorDriverRegistry, keeping this class's job to just own the
    // config value, not drive hardware.
    void setMotorDriver(uint8_t driverId, bool vehicleArmed);

private:
    Preferences _prefs;
    String _name;
    MotorDriverType _driver;
    OnNameOkCallback _onNameOk;
    OnDriverOkCallback _onDriverOk;
    OnVesselConfigErrorCallback _onError;
};
