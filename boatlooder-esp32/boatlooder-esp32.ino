#include "Arduino.h"
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "BluetoothHandler.h"
#include "Mavlink.h"

// TMC2225
#define EN_PIN           23
#define DIR_PIN          5
#define TMC_SERIAL_PORT  Serial2
#define R_SENSE          0.11f
#define STEP_PIN         18

#define MOTOR_ANGLE_PER_STEP 1.8f
#define MOTOR_GEAR_RATIO 4
#define MICROSTEPS       4

#define ROTATION_ANGLE_MAX 90.0
#define STEPPER_DEAD_BAND 15
#define STEPPER_MID_DEAD_BAND 10
#define STEPS_FOR_90_DEGREES (int)((ROTATION_ANGLE_MAX / MOTOR_ANGLE_PER_STEP) * MICROSTEPS * MOTOR_GEAR_RATIO)

// general defines
#define PWM_MID 1500
#define PWM_MIN 1100
#define PWM_MAX 1900

// define motor driver
//#define DRIVER_POLULU_18V17
#define DRIVER_BTS7960

// motor driver pin defines
#ifdef DRIVER_POLULU_18V17
#define MOTOR_DIR_PIN 21
#define MOTOR_PWM_PIN 19
#elif defined(DRIVER_BTS7960)
#define MOTOR_PWM1_PIN 21
#define MOTOR_PWM2_PIN 22
#define MOTOR_EN_PIN 19
#endif
//#define CURRENT_SENSE_PIN 34

#define MOTOR_DEADBAND 20
#define MOTOR_LOWER_BOUND (PWM_MID - MOTOR_DEADBAND)
#define MOTOR_UPPER_BOUND (PWM_MID + MOTOR_DEADBAND)


// rc settings
#define NUM_RC_CHANNELS 5

#define MAVLINK_UART 1
 
// Commonly, the built-in LED is on GPIO 2 for ESP32 DevKit boards
#define LED_PIN 2

// GLOBALS

// Stepper driver initialization
TMC2208Stepper driver = TMC2208Stepper(&TMC_SERIAL_PORT, R_SENSE);
AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Bluetooth initialization
BluetoothHandler btHandler;

// Mavlink initialization
Mavlink mavlink(NUM_RC_CHANNELS, MAVLINK_UART);
uint16_t rcChannels[NUM_RC_CHANNELS];

// function prototypes
int pwmToSteps(int pwm);
void onBluetoothWrite(const uint8_t* data, size_t length);
void onBluetoothConnect();
void onBluetoothDisconnect();

// TASKS

//  stepper driver control task
void stepperControlTask(void *pvParameters) {
  uint16_t lastStepperPulseUs = 1500;
  //int target = 0;
  unsigned long lastUpdate = 0;

  while (1) {
    
    uint16_t stepperPulseUs = mavlink.getSteeringPulseUs();

    if( (stepperPulseUs != lastStepperPulseUs) &&
        abs(stepperPulseUs - lastStepperPulseUs) > STEPPER_DEAD_BAND ) { 
          int target = pwmToSteps(stepperPulseUs);
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

// thrust control 
void setupMotorPWM() {
#ifdef DRIVER_POLULU_18V17
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  ledcAttachChannel(MOTOR_PWM_PIN, 20000, 9, 0);
  ledcWrite(MOTOR_PWM_PIN, 0);
#elif defined(DRIVER_BTS7960)
  pinMode(MOTOR_EN_PIN, OUTPUT);
  pinMode(MOTOR_PWM1_PIN, OUTPUT);
  pinMode(MOTOR_PWM2_PIN, OUTPUT);
  
  digitalWrite(MOTOR_EN_PIN, HIGH);

  ledcAttachChannel(MOTOR_PWM1_PIN, 20000, 9, 0);
  ledcAttachChannel(MOTOR_PWM2_PIN, 20000, 9, 1);
  ledcWrite(MOTOR_PWM2_PIN, 0);
#endif
}

void setMotorSpeed(int motorPulse) {
  bool direction = motorPulse > PWM_MID;
  int speed;
  
#ifdef DRIVER_POLULU_18V17
  digitalWrite(MOTOR_DIR_PIN, direction ? HIGH : LOW);
  if (direction) {
    speed = map(motorPulse, PWM_MID, PWM_MAX, 0, 512);
  } else {
    speed = map(motorPulse, PWM_MID, PWM_MIN, 0, 512);
  }
  speed = constrain(speed, 0, 512);
  if (motorPulse > (PWM_MID - MOTOR_DEADBAND) && motorPulse < (PWM_MID + MOTOR_DEADBAND)) {
    speed = 0;
  }
  ledcWrite(MOTOR_PWM_PIN, speed);
#elif defined(DRIVER_BTS7960)
  if (motorPulse > (PWM_MID + MOTOR_DEADBAND)) {
    ledcWrite(MOTOR_PWM1_PIN, 0);
    speed = map(motorPulse, PWM_MID, PWM_MAX, 0, 512);
    speed = constrain(speed, 0, 512);
    ledcWrite(MOTOR_PWM2_PIN, speed);
  } else if (motorPulse < (PWM_MID - MOTOR_DEADBAND)) {
    ledcWrite(MOTOR_PWM2_PIN, 0);
    speed = map(motorPulse, PWM_MID, PWM_MIN, 0, 512);
    speed = constrain(speed, 0, 512);
    ledcWrite(MOTOR_PWM1_PIN, speed);
  } else {
    ledcWrite(MOTOR_PWM1_PIN, 0);
    ledcWrite(MOTOR_PWM2_PIN, 0);
  }
#endif
}

void thrustControlTask(void *pvParameters) {
  int lastThrustPulseUs = 1500;
  setupMotorPWM();

  while (1) {
    int thrustPulseUs = mavlink.getThrottlePulseUs();

    if(thrustPulseUs != lastThrustPulseUs){
      setMotorSpeed(thrustPulseUs);
      lastThrustPulseUs = thrustPulseUs;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void processMavlinkTask(void *pvParameters) {
  while (1) {
    mavlink.processReceivedPacket();
    mavlink.sendRcOverrides((const uint16_t *) rcChannels);
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait for 50 milliseconds
  }
}

// Task function to toggle the LED
void toggleLED(void *parameter) {
  pinMode(LED_PIN, OUTPUT); // Set the LED pin as an output

  while (1) {
    digitalWrite(LED_PIN, HIGH); // Toggle the LED
    vTaskDelay(pdMS_TO_TICKS(1000));
    digitalWrite(LED_PIN, LOW); // Toggle the LED
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("\nstart...");

  // init mavlink
  mavlink.init();
  initRcChannels();

  // stepper driver initialization
  TMC_SERIAL_PORT.begin(115200, SERIAL_8N1, 16, 17);
  while(!TMC_SERIAL_PORT);
  
  driver.begin();
  driver.rms_current(800);
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
  xTaskCreatePinnedToCore(toggleLED, "Toggle LED", 1024, NULL, 1, NULL, 0);
  // in order to achieve fast updates stepper control task runs at priority 0 with idle task
  // using taskYield in tasks with higher priority would starve the idle task and trigger the watchdog timeout
  xTaskCreatePinnedToCore(stepperControlTask, "Control Stepper", 4096, NULL, 0, NULL, 1); // give stepper single core
  xTaskCreatePinnedToCore(thrustControlTask, "Control Thrust Motor", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(processMavlinkTask, "Process Mavlink", 4096, NULL, 1, NULL, 0);

  btHandler.init();  // Initialize Bluetooth
  // set bluetooth callbacks
  btHandler.setOnWriteCallback(onBluetoothWrite);
  btHandler.setOnConnectCallback(onBluetoothConnect);
  btHandler.setOnDisconnectCallback(onBluetoothDisconnect);
}

void loop() {
  // if this line is being hit something seriously went wrong
}

///////////////// CALLBACKS /////////////////

void onBluetoothWrite(const uint8_t* data, size_t length) {
    //Serial.printf("Received %d bytes via BLE\n", length);
    if(length == 8){
      rcChannels[0] = data[2] << 8 | data[3]; // channel1 (roll)
      rcChannels[1] = 1500;                   // constant value (pitch)
      rcChannels[2] = data[4] << 8 | data[5]; // channel3 (throttle)
      rcChannels[3] = data[6] << 8 | data[7]; // channel4 (arm/disarm)
      rcChannels[4] = data[0] << 8 | data[1]; // channel5 (mode)
    }
}

void onBluetoothConnect() {
    Serial.println("BLE device connected");
    stepper.enableOutputs();
}

void onBluetoothDisconnect() {
    Serial.println("BLE device disconnected");

    initRcChannels(); //set all channels to < 900us to trigger failsage
    stepper.disableOutputs();
#ifdef DRIVER_POLULU_18V17
  ledcWrite(MOTOR_PWM_PIN, 0);
#elif defined(DRIVER_BTS7960)
  ledcWrite(MOTOR_PWM1_PIN, 0);
  ledcWrite(MOTOR_PWM2_PIN, 0);
#endif
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

void initRcChannels(){
  for (int i = 0; i < NUM_RC_CHANNELS; i++) {
    rcChannels[i] = 1500;
  }
  rcChannels[3] = 1100; //disarmed
  rcChannels[4] = 1100; //manual mode
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
//     // Serial.print("Channel 1: "); Serial.println(channelDefaults[0]);
//     // Serial.print("Channel 2: "); Serial.println(channelDefaults[1]);
//     vTaskDelay(pdMS_TO_TICKS(500));
//   }
// }