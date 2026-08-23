#include "Arduino.h"
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "BluetoothHandler.h"
#include "Mavlink.h"
#include "OtaUpdate.h"
#include "VesselConfig.h"
#include "motor_drivers/MotorDriverBackend.h"
#include "motor_drivers/MotorDriverRegistry.h"
#include "Logger.h"

// Bump on every firmware release pushed over OTA -- reported on the
// ota_version characteristic (see BluetoothHandler::init), and what the
// app's OtaController compares against its own bundled version to decide
// "up to date" (exact string match, no semver parsing -- see that file).
#define FIRMWARE_VERSION "1.1.0"

// TMC2225
#define EN_PIN                     23
#define DIR_PIN                    5
#define TMC_SERIAL_PORT            Serial2
#define R_SENSE                    0.11f
#define STEP_PIN                   18
#define CURRENT_MILLI_AMPS         1000

#define MOTOR_ANGLE_PER_STEP       1.8f
#define DRIVE_TEETH                16
#define OUTPUT_TEETH               90
#define MOTOR_GEAR_RATIO           ((float)OUTPUT_TEETH / (float)DRIVE_TEETH)
#define MICROSTEPS                 4

#define ROTATION_ANGLE_MAX         90.0f
#define STEPPER_DEAD_BAND          5
#define STEPPER_MID_DEAD_BAND      10
#define STEPS_FOR_90_DEGREES       ((int)(((ROTATION_ANGLE_MAX / MOTOR_ANGLE_PER_STEP) * MICROSTEPS * MOTOR_GEAR_RATIO) + 0.5f))

// general defines
// PWM_MID/PWM_MIN/PWM_MAX/MOTOR_DEADBAND now live in
// motor_drivers/MotorDriverBackend.h -- shared between main.cpp and every
// backend .cpp. Which motor driver is active is a runtime choice now
// (VesselConfig, see below), not a compile-time #define -- both backends
// are always compiled in, see motor_drivers/.
//#define CURRENT_SENSE_PIN 34

// rc settings
#define NUM_RC_CHANNELS            5

#define MAVLINK_UART               1
 
// Commonly, the built-in LED is on GPIO 2 for ESP32 DevKit boards
#define LED_PIN                    2

// GLOBALS

// Stepper driver initialization
TMC2208Stepper driver = TMC2208Stepper(&TMC_SERIAL_PORT, R_SENSE);
AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Bluetooth initialization
BluetoothHandler btHandler;

// Mavlink initialization
Mavlink mavlink(NUM_RC_CHANNELS, MAVLINK_UART);
uint16_t rcChannels[NUM_RC_CHANNELS];

// OTA firmware update state machine -- see OtaUpdate.h.
OtaUpdate otaUpdate;

// Persisted vessel name + motor driver choice -- see VesselConfig.h.
VesselConfig vesselConfig;

// The motor driver backend thrustControlTask is currently driving --
// global (not local to that task) so onBluetoothDisconnect() can also
// reach it to zero the motor on a dropped link, the same way it always
// has. Only thrustControlTask ever changes which backend this points at.
MotorDriverBackend* activeMotorDriver = nullptr;

// function prototypes
// (the Arduino IDE used to generate these implicitly from the .ino file)
int pwmToSteps(int pwm);
void initRcChannels();
void stepperControlTask(void *pvParameters);
void thrustControlTask(void *pvParameters);
void processMavlinkTask(void *pvParameters);
void statusLedTask(void *parameter);
void onBluetoothWrite(const uint8_t* data, size_t length);
void onBluetoothOtaControlWrite(const uint8_t* data, size_t length);
void onBluetoothOtaDataWrite(const uint8_t* data, size_t length);
void onBluetoothVesselConfigWrite(const uint8_t* data, size_t length);
void rebootTask(void *pvParameters);
void onBluetoothSettingsWrite(const uint8_t* data, size_t length);
void onBluetoothConnect();
void onBluetoothDisconnect();

// TASKS

//  stepper driver control task
void stepperControlTask(void *pvParameters) {
  uint16_t lastStepperPulseUs = 1500;
  //int target = 0;
  unsigned long lastUpdate = 0;
  LOG_INFO("Stepper initialiazed");

  while (1) {
    
    uint16_t stepperPulseUs = mavlink.getSteeringPulseUs();

    if( (stepperPulseUs != lastStepperPulseUs) &&
        abs(stepperPulseUs - lastStepperPulseUs) > STEPPER_DEAD_BAND ) { 
          int target = pwmToSteps(stepperPulseUs);
          LOG_DEBUGF("Stepper pulse: %d\n", stepperPulseUs);
          stepper.moveTo(target);  // Move stepper to the new target
          lastStepperPulseUs = stepperPulseUs;  // Update the old target
    }
     // Continuously run the stepper to hit target
    stepper.run();
    // taskYield immediately gives away control to anoter task with same or
    // higher priority but task must be on same priority as idle task to not 
    // fire watchdog 
    taskYIELD();
  }
}

// thrust control -- driver choice is a runtime value (vesselConfig,
// checked every loop below), not compiled in. See motor_drivers/.
void thrustControlTask(void *pvParameters) {
  int lastThrustPulseUs = 1500;
  activeMotorDriver = getMotorDriverBackend(vesselConfig.currentMotorDriver());
  activeMotorDriver->begin();

  while (1) {
    // A driver change from a SET_DRIVER write (VesselConfig::setMotorDriver(),
    // already gated on disarmed there) is picked up here, not applied from
    // the BLE callback's own context -- this task is the one place that
    // owns motor output, so the actual pin reconfiguration only ever runs
    // here. Quiesce -> detach -> reconfigure -> attach, via end()/begin().
    MotorDriverType desired = vesselConfig.currentMotorDriver();
    if (desired != activeMotorDriver->type()) {
      LOG_INFOF("Switching motor driver to %s\n", getMotorDriverBackend(desired)->name());
      activeMotorDriver->end();
      activeMotorDriver = getMotorDriverBackend(desired);
      activeMotorDriver->begin();
    }

    int thrustPulseUs = mavlink.getThrottlePulseUs();
    //LOG_DEBUGF("Thrust: %d\n", thrustPulseUs);

    if(thrustPulseUs != lastThrustPulseUs){
      activeMotorDriver->setSpeed(thrustPulseUs);
      lastThrustPulseUs = thrustPulseUs;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void processMavlinkTask(void *pvParameters) {
  bool deviceConnected = false; // Track if the opposite heartbeat device is connected

  while (1) {
    mavlink.processReceivedPackets();

    if (mavlink.haveHeartbeat()) {
      if (!deviceConnected) {
        // Device connected for the first time or after a loss
        mavlink.setupStreamingRates();
        deviceConnected = true;
        LOG_INFO("Received Mavlink Heartbeat");
      }
      // Confirm the requested telemetry streams actually turned up, and chase
      // any that did not -- SET_MESSAGE_INTERVAL is not acknowledged reliably.
      mavlink.ensureStreamsFlowing();

      // Send RC overrides since the device is connected
      mavlink.sendRcOverrides((const uint16_t *) rcChannels);
    } else {
      // Device disconnected, set flag to false
      deviceConnected = false;
    }

    vTaskDelay(pdMS_TO_TICKS(50)); // Wait for 50 milliseconds
  }
}

// Task function to toggle the LED
void statusLedTask(void *parameter) {
  pinMode(LED_PIN, OUTPUT); // Set the LED pin as an output
  LOG_DEBUG("LED initialiazed");


  while (1) {
    if (mavlink.haveHeartbeat()) {
      digitalWrite(LED_PIN, HIGH); // Keep the LED on
    } else {
      static int ledState = HIGH;
      digitalWrite(LED_PIN, ledState);
      ledState = !ledState;
    }
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay for half a second
  }
}

void setup() {
  // initialize serial for logger
  Serial.begin(115200);
  while(!Serial);
  LOG_INFO("\nstart...");

  // init mavlink
  mavlink.init();
  initRcChannels();

  // Loads the persisted name/driver choice before anything reads them --
  // in particular, thrustControlTask (started below) reads
  // vesselConfig.currentMotorDriver() the moment it starts running on its
  // own core, which can happen before the rest of setup() continues.
  vesselConfig.init();

  // stepper driver initialization
  TMC_SERIAL_PORT.begin(115200, SERIAL_8N1, 16, 17);
  while(!TMC_SERIAL_PORT);
  
  driver.begin();
  driver.rms_current(CURRENT_MILLI_AMPS);
  driver.microsteps(MICROSTEPS);
  driver.en_spreadCycle(false);
  driver.ihold(7);

  // AccelStepper initialization
  stepper.setMaxSpeed(5000);
  stepper.setAcceleration(2000); 
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true);
  stepper.disableOutputs();

  // task init
  xTaskCreatePinnedToCore(statusLedTask, "LED Status", 1024, NULL, 1, NULL, 0);
  // in order to achieve fast updates stepper control task runs at priority 0 with idle task
  // using taskYield in tasks with higher priority would starve the idle task and trigger the watchdog timeout
  xTaskCreatePinnedToCore(stepperControlTask, "Control Stepper", 4096, NULL, 1, NULL, 1); // give stepper single core
  xTaskCreatePinnedToCore(thrustControlTask, "Control Thrust Motor", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(processMavlinkTask, "Process Mavlink", 4096, NULL, 1, NULL, 0);

  // Full advertised name = APP_FILTER_PREFIX "@" + the persisted (or
  // default) bare vessel name -- see VesselConfig.h. boatlooder-app's
  // BleController.deviceNamePrefix must match APP_FILTER_PREFIX exactly.
  String deviceName = String(APP_FILTER_PREFIX) + "@" + vesselConfig.currentName();
  btHandler.init(FIRMWARE_VERSION, deviceName.c_str());  // Initialize Bluetooth
  btHandler.setVesselInfo(vesselConfig.currentName().c_str(),
                           getMotorDriverBackend(vesselConfig.currentMotorDriver())->name());
  // set bluetooth callbacks
  btHandler.setOnWriteCallback(onBluetoothWrite);
  btHandler.setOnSettingsWriteCallback(onBluetoothSettingsWrite);
  btHandler.setOnOtaControlWriteCallback(onBluetoothOtaControlWrite);
  btHandler.setOnOtaDataWriteCallback(onBluetoothOtaDataWrite);
  btHandler.setOnVesselConfigWriteCallback(onBluetoothVesselConfigWrite);
  btHandler.setOnConnectCallback(onBluetoothConnect);
  btHandler.setOnDisconnectCallback(onBluetoothDisconnect);

  // OtaUpdate knows nothing about BLE -- wire its outcomes to
  // ota_control notify frames here. ACK/OK/ERROR match OtaUpdate.h's
  // doc comment on the wire layout; keep boatlooder-app's OtaController
  // in sync with any change here.
  otaUpdate.setOnAckCallback([]() {
    uint8_t frame[1] = {0x01};
    btHandler.notifyOtaControl(frame, sizeof(frame));
  });
  otaUpdate.setOnOkCallback([]() {
    uint8_t frame[1] = {0x02};
    btHandler.notifyOtaControl(frame, sizeof(frame));
    // Reboot off a separate task, not inline here: notify() only queues
    // the packet, it doesn't wait for the radio to actually send it --
    // restarting synchronously in this same callback risks tearing the
    // BLE stack down before that last notify ever goes out.
    xTaskCreate(rebootTask, "reboot", 2048, nullptr, 1, nullptr);
  });
  otaUpdate.setOnErrorCallback([](OtaError error) {
    uint8_t frame[2] = {0x03, (uint8_t)error};
    btHandler.notifyOtaControl(frame, sizeof(frame));
  });
  // VesselConfig knows nothing about BLE -- wire its outcomes to
  // vessel_config notify frames here, same shape as OtaUpdate's wiring
  // above. Keep boatlooder-app's VesselConfigController in sync with any
  // change here.
  vesselConfig.setOnNameOkCallback([]() {
    uint8_t frame[1] = {0x01};
    btHandler.notifyVesselConfig(frame, sizeof(frame));
    // Keep vessel_info's READ value in sync -- the *advertised* BLE name
    // stays whatever it was at this boot (BLEDevice::init() only runs
    // once), but vessel_info's "name=" field should reflect the new
    // value immediately, same as the app's own rename field does, with
    // "applied on next boot" being about the advertised name specifically.
    btHandler.setVesselInfo(vesselConfig.currentName().c_str(),
                             getMotorDriverBackend(vesselConfig.currentMotorDriver())->name());
  });
  vesselConfig.setOnDriverOkCallback([]() {
    uint8_t frame[1] = {0x02};
    btHandler.notifyVesselConfig(frame, sizeof(frame));
    // Keep vessel_info's READ value in sync with a live driver switch too.
    btHandler.setVesselInfo(vesselConfig.currentName().c_str(),
                             getMotorDriverBackend(vesselConfig.currentMotorDriver())->name());
  });
  vesselConfig.setOnErrorCallback([](VesselConfigError error) {
    uint8_t frame[2] = {0x03, (uint8_t)error};
    btHandler.notifyVesselConfig(frame, sizeof(frame));
  });
  // relay mavlink coming off the UART out to the app
  mavlink.setOnTelemetryRelayCallback([](const uint8_t* data, size_t len) {
    btHandler.notifyTelemetry(data, len);
  });
  mavlink.setOnSettingsAckRelayCallback([](const uint8_t* data, size_t len) {
    btHandler.notifySettings(data, len);
  });
}

void loop() {
  // if this line is being hit something seriously went wrong
}

///////////////// CALLBACKS /////////////////

// Two 3-byte frame shapes share this characteristic, told apart by the
// leading tag byte -- both are the same length, so length alone can't
// distinguish them. 0x01 is an axis update (throttle/rudder, sent on every
// drag update, compact int8s so it can go out without waiting for a GATT
// response). 0x02 is a command update (mode/arm, sent only on a deliberate
// change, written with response since a dropped one-shot command has
// nothing else to resend it). See boatlooder-app's BleController for the
// encoding this decodes.
void onBluetoothWrite(const uint8_t* data, size_t length) {
    if (length != 3) return;

    switch (data[0]) {
      case 0x01: { // axis: throttle_pct (int8 -100..100), rudder_deg (int8 -90..90)
        int8_t throttlePercent = (int8_t)data[1];
        int8_t rudderDeg = (int8_t)data[2];
        rcChannels[0] = map(throttlePercent, -100, 100, PWM_MIN, PWM_MAX); // throttle
        rcChannels[2] = map(rudderDeg, -90, 90, PWM_MIN, PWM_MAX);         // rudder
        break;
      }
      case 0x02: { // command: mode_id (ArduPilot custom_mode), arm_state (0/1)
        uint8_t modeId = data[1];
        uint8_t armState = data[2];
        rcChannels[3] = armState ? PWM_MAX : PWM_MIN; // arm/disarm -- still an RC override
        mavlink.setMode(modeId);                       // mode -- a real MAVLink command now
        break;
      }
      default:
        break; // unrecognized tag
    }
}

// mavlink written by the app on the settings characteristic, fed byte-wise into
// the second parser channel (the allowlist lives in Mavlink::handleBleSettingsByte)
void onBluetoothSettingsWrite(const uint8_t* data, size_t length) {
    for (size_t i = 0; i < length; i++) {
      mavlink.handleBleSettingsByte(data[i]);
    }
}

// BEGIN/END/ABORT on ota_control -- see OtaUpdate.h for the frame layout
// this decodes and boatlooder-app's OtaController for the sending side.
void onBluetoothOtaControlWrite(const uint8_t* data, size_t length) {
    if (length < 1) return;

    switch (data[0]) {
      case 0x01: { // BEGIN: size (u32 LE), verLen (u8), verBytes[verLen] -- version unused here, logged only
        if (length < 5) return;
        uint32_t size = (uint32_t)data[1] | ((uint32_t)data[2] << 8) |
                         ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 24);
        otaUpdate.begin(size);
        break;
      }
      case 0x02: // END
        otaUpdate.end();
        break;
      case 0x03: // ABORT
        otaUpdate.abort();
        break;
      default:
        break; // unrecognized tag
    }
}

// SET_NAME/SET_DRIVER on vessel_config -- see VesselConfig.h for the frame
// layout this decodes and boatlooder-app's VesselConfigController for the
// sending side.
void onBluetoothVesselConfigWrite(const uint8_t* data, size_t length) {
    if (length < 1) return;

    switch (data[0]) {
      case 0x01: { // SET_NAME: nameLen (u8), nameBytes[nameLen]
        if (length < 2) return;
        uint8_t nameLen = data[1];
        if (length < (size_t)(2 + nameLen)) return;
        vesselConfig.setName(data + 2, nameLen);
        break;
      }
      case 0x02: { // SET_DRIVER: driverId (u8)
        if (length < 2) return;
        vesselConfig.setMotorDriver(data[1], mavlink.isArmed());
        break;
      }
      default:
        break; // unrecognized tag
    }
}

// Raw firmware bytes -- fed straight to OtaUpdate in the order received.
// No framing of its own: write-with-response already serializes delivery
// order (see BluetoothHandler's ota_data characteristic doc comment).
void onBluetoothOtaDataWrite(const uint8_t* data, size_t length) {
    otaUpdate.writeChunk(data, length);
}

// Gives the OK notify a moment to actually clear the BLE link before
// tearing the stack down for the reboot -- see where this is spawned in
// setup().
void rebootTask(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP.restart();
}

void onBluetoothConnect() {
    LOG_INFO("BLE device connected");
    stepper.enableOutputs();
}

void onBluetoothDisconnect() {
    LOG_INFO("BLE device disconnected");

    // A dropped link mid-transfer must not leave a half-written image
    // sitting in the inactive OTA slot looking like it could still be
    // finished later -- see OtaUpdate::abort()'s doc comment. No-op if
    // nothing was in progress.
    otaUpdate.abort();

    initRcChannels(); //set all channels to < 900us to trigger failsage
    stepper.disableOutputs();
    if (activeMotorDriver != nullptr) {
      activeMotorDriver->setSpeed(PWM_MID); // neutral -- zero speed on whichever driver is active
    }
}

///////////////// HELPER /////////////////

// Function to calculate the number of steps for a given PWM value
int pwmToSteps(int pwmValue) {
    // Normalize the PWM value to the range -1 to 1
    float normalizedPWM = (float)(pwmValue - PWM_MID) / (pwmValue < PWM_MID ? PWM_MID - PWM_MIN : PWM_MAX - PWM_MID);
    normalizedPWM = constrain(normalizedPWM, -1.0, 1.0); // Ensure it's within the range
    
    // Calculate the number of steps based on the normalized PWM value
    int steps = round(normalizedPWM * STEPS_FOR_90_DEGREES);
    
    return steps;
}

void initRcChannels() {
  for (int i = 0; i < NUM_RC_CHANNELS; i++) {
    rcChannels[i] = 1500;
  }
  rcChannels[3] = 1100; //disarmed
  // rcChannels[4] (the old mode channel) stays at the neutral 1500 the loop
  // above already gave it -- mode now goes out via Mavlink::setMode()'s
  // real MAV_CMD_DO_SET_MODE, not by simulating an RC channel PWM band, so
  // this slot is permanently inert. Left in the fixed 5-channel array
  // rather than removed; sendRcOverrides() still expects 5 pulses.
}

// currently unused
// read analog channels and provide them to ppm timer interrupt
// void readAnalogTask(void *pvParameters) {
//   // pin setup
//   pinMode(PPM_PIN, OUTPUT);
//   digitalWrite(PPM_PIN, LOW);
//   pinMode(THROTTLE_INPUT_PIN, INPUT);  // Set pin for throttle input
//   pinMode(RUDDER_INPUT_PIN, INPUT);    // Set pin for rudder input

//   // timer initialization
//   timer = timerBegin(TIMER_NUMBER, PRESCALER, true);
//   timerAttachInterrupt(timer, &handlePpmInterrupt, true);
//   timerAlarmWrite(timer, FRAME_LENGTH, true);
//   timerAlarmEnable(timer);

//   while(1) {
//     channelDefaults[0] = map(analogRead(THROTTLE_INPUT_PIN), 0, 4095, PPM_MIN, PPM_MAX);
//     channelDefaults[1] = map(analogRead(RUDDER_INPUT_PIN), 0, 4095, PPM_MIN, PPM_MAX);
//     vTaskDelay(pdMS_TO_TICKS(500));
//   }
// }