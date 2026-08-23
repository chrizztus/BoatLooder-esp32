#include "VesselConfig.h"
#include "Logger.h"

VesselConfig::VesselConfig() : _name(DEFAULT_VESSEL_NAME), _driver(MotorDriverType::Bts7960) {}

void VesselConfig::setOnNameOkCallback(OnNameOkCallback callback) { _onNameOk = callback; }
void VesselConfig::setOnDriverOkCallback(OnDriverOkCallback callback) { _onDriverOk = callback; }
void VesselConfig::setOnErrorCallback(OnVesselConfigErrorCallback callback) { _onError = callback; }

// Kept open for the app's lifetime rather than begin()/end() per access --
// this namespace is small and read far more than written, no need to pay
// the reopen cost on every currentName()/currentMotorDriver() call.
void VesselConfig::init() {
    _prefs.begin("boatlooder", false);
    _name = _prefs.getString("name", DEFAULT_VESSEL_NAME);
    _driver = (MotorDriverType)_prefs.getUChar("driver", (uint8_t)MotorDriverType::Bts7960);
    LOG_INFOF("Vessel config loaded: name='%s', driver=%u\n", _name.c_str(), (unsigned)_driver);
}

void VesselConfig::setName(const uint8_t* data, size_t length) {
    if (length == 0 || length > VESSEL_NAME_MAX_LEN) {
        LOG_WARNF("Vessel name rejected: %u bytes\n", (unsigned)length);
        if (_onError) _onError(VesselConfigError::NAME_TOO_LONG);
        return;
    }
    String name;
    name.reserve(length);
    for (size_t i = 0; i < length; i++) {
        name += (char)data[i];
    }
    _name = name;
    _prefs.putString("name", _name);
    LOG_INFOF("Vessel name set to '%s' (applies on next boot)\n", _name.c_str());
    if (_onNameOk) _onNameOk();
}

void VesselConfig::setMotorDriver(uint8_t driverId, bool vehicleArmed) {
    // The real safety boundary -- re-wiring motor control pins while the
    // vessel could be powered and moving is a physical hazard, not just a
    // UX nicety to gate app-side.
    if (vehicleArmed) {
        LOG_WARN("Motor driver change rejected: vehicle armed");
        if (_onError) _onError(VesselConfigError::ARMED);
        return;
    }
    if (driverId != (uint8_t)MotorDriverType::Pololu18v17 &&
        driverId != (uint8_t)MotorDriverType::Bts7960) {
        LOG_WARNF("Motor driver change rejected: invalid id %u\n", (unsigned)driverId);
        if (_onError) _onError(VesselConfigError::INVALID_DRIVER);
        return;
    }
    _driver = (MotorDriverType)driverId;
    _prefs.putUChar("driver", driverId);
    LOG_INFOF("Motor driver set to %u\n", (unsigned)driverId);
    if (_onDriverOk) _onDriverOk();
}
