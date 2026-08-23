#include "OtaUpdate.h"
#include "Logger.h"
#include <Update.h>

OtaUpdate::OtaUpdate() : _inProgress(false), _expectedSize(0), _written(0) {}

void OtaUpdate::setOnAckCallback(OnOtaAckCallback callback) { _onAck = callback; }
void OtaUpdate::setOnOkCallback(OnOtaOkCallback callback) { _onOk = callback; }
void OtaUpdate::setOnErrorCallback(OnOtaErrorCallback callback) { _onError = callback; }

void OtaUpdate::begin(uint32_t size) {
    if (_inProgress) {
        LOG_WARN("OTA begin() while an update is already in progress");
        if (_onError) _onError(OtaError::ALREADY_IN_PROGRESS);
        return;
    }
    if (size == 0) {
        LOG_WARN("OTA begin() with a zero size");
        if (_onError) _onError(OtaError::INVALID_SIZE);
        return;
    }
    // Finds and prepares the *inactive* OTA app slot (min_spiffs.csv gives
    // two, see platformio.ini) sized for `size` -- fails here, not later,
    // if the declared size doesn't fit what's actually available.
    if (!Update.begin(size)) {
        LOG_ERRORF("OTA Update.begin(%u) failed: %s\n", (unsigned)size,
                   Update.errorString());
        if (_onError) _onError(OtaError::BEGIN_FAILED);
        return;
    }
    _inProgress = true;
    _expectedSize = size;
    _written = 0;
    LOG_INFOF("OTA begin: %u bytes expected\n", (unsigned)size);
    if (_onAck) _onAck();
}

void OtaUpdate::writeChunk(const uint8_t* data, size_t length) {
    if (!_inProgress) {
        LOG_WARN("OTA data received with no update in progress");
        if (_onError) _onError(OtaError::NOT_STARTED);
        return;
    }
    size_t written = Update.write(const_cast<uint8_t*>(data), length);
    if (written != length) {
        LOG_ERRORF("OTA write failed: wrote %u of %u bytes (%s)\n",
                   (unsigned)written, (unsigned)length, Update.errorString());
        // A short write leaves the inactive slot in a state a later
        // begin() must not inherit -- discard it now rather than let the
        // app's eventual END/ABORT be the only thing standing between
        // this and a corrupt image looking complete.
        Update.abort();
        _inProgress = false;
        if (_onError) _onError(OtaError::WRITE_FAILED);
        return;
    }
    _written += length;
}

void OtaUpdate::end() {
    if (!_inProgress) {
        LOG_WARN("OTA end() with no update in progress");
        if (_onError) _onError(OtaError::NOT_STARTED);
        return;
    }
    _inProgress = false;
    // Update.end(true) checks both the declared size (tracked internally
    // since begin()) and the image's own embedded checksum -- a short or
    // corrupt transfer fails here, not by silently booting into garbage.
    if (!Update.end(true) || !Update.isFinished()) {
        LOG_ERRORF("OTA end() failed: %s\n", Update.errorString());
        if (_onError) _onError(OtaError::END_FAILED);
        return;
    }
    LOG_INFOF("OTA complete: %u bytes written and verified\n", (unsigned)_written);
    if (_onOk) _onOk();
}

void OtaUpdate::abort() {
    if (!_inProgress) return;
    LOG_WARN("OTA aborted");
    Update.abort();
    _inProgress = false;
}
