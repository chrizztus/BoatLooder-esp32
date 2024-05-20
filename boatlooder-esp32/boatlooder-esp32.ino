#include "Arduino.h"
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "BluetoothHandler.h"

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
#define TX_CHANNELS 5
#define RC_TX_PIN 14
#define RC_RX_PIN 27

//#define USE_SBUS
#ifdef USE_SBUS
#include "sbus.h"
#else
#include "Ppm.h"
#endif

//#define UART_STEPPER_TEST_MODE 

const int ledPin = 2; // Commonly, the built-in LED is on GPIO 2 for ESP32 DevKit boards
const int thrustPwmPin = 13; // PWM Input Pin
const int stepperPwmPin = 12; // PWM Input Pin

// Stepper driver initialization
TMC2208Stepper driver = TMC2208Stepper(&TMC_SERIAL_PORT, R_SENSE);
AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Bluetooth initialization
BluetoothHandler btHandler;

#ifdef USE_SBUS
#include "sbus.h"
bfs::SbusTx sbusTx(&Serial1, RC_RX_PIN, RC_TX_PIN, true);
bfs::SbusData sbusData;
#else
Ppm ppm(TX_CHANNELS, RC_TX_PIN);
#endif


// function prototypes
void rcTransmit(const int* channels);
int pwmToSteps(int pwm);
void onBluetoothWrite(const uint8_t* data, size_t length);
void onBluetoothConnect();
void onBluetoothDisconnect();

// PWM input pin definition
struct PwmPinData {
  volatile unsigned long lastRisingEdgeTime = 0;
  volatile unsigned long pulseDuration = 0;
  uint8_t pin;
  
  PwmPinData(uint8_t pin): pin(pin) {}
};

volatile PwmPinData thrustPinData(thrustPwmPin); // Initialize pwm pin data for the thrust input
volatile PwmPinData stepperPinData(stepperPwmPin); // Initialize pwm pin data for the stepper input

// INTERRUPTS

// thrust/rudder input pwm reading
void IRAM_ATTR handlePwmInterrupt(void *arg) {
  PwmPinData *pinData = static_cast<PwmPinData*>(arg);
  unsigned long currentTime = micros(); // Current timestamp in microseconds
  // measure duration of HIGH pulse
  if (digitalRead(pinData->pin) == HIGH) { // If the current edge is rising
    pinData->lastRisingEdgeTime = currentTime; // Update the last rising edge timestamp
  } else { // Falling edge
    if (pinData->lastRisingEdgeTime  > 0) { // Ensure there was a previous rising edge
      pinData->pulseDuration = currentTime - pinData->lastRisingEdgeTime; // Calculate the pulse duration
      pinData->lastRisingEdgeTime = 0; // Reset the rising edge timestamp
    }
  }
}

// TASKS

//  stepper driver control task
void stepperControlTask(void *pvParameters) {
  int oldPulse = 1500;
  //int target = 0;
  unsigned long lastUpdate = 0;

  stepperPinData.pulseDuration = 1500;
  while (1) {
    //if (stepper.distanceToGo() == 0) {
    noInterrupts();
    int stepperPulse = stepperPinData.pulseDuration;
    interrupts();

    if (abs(stepperPulse - oldPulse) > STEPPER_DEAD_BAND) { //&&
            //(abs(stepperPulse - PWM_MID) > STEPPER_MID_DEAD_BAND)) {
            int target = pwmToSteps(stepperPinData.pulseDuration);
            stepper.moveTo(target);  // Move stepper to the new target
            oldPulse = stepperPulse;  // Update the old target
    }
    // Continuously run the stepper to hit target
    stepper.run();
    // taskYield immediately gives away control to anoter task with same or
    // higher priority but task must be on same priority as idle task to not 
    // fire watchdog 
    taskYIELD();
  }
}

// thrust control task
#ifdef DRIVER_POLULU_18V17
void thrustControlTask(void *pvParameters) {
  // pwm inits
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  ledcAttachChannel(MOTOR_PWM_PIN, 20000, 9, 0);
  ledcWrite(MOTOR_PWM_PIN, 0);
  thrustPinData.pulseDuration = 1500;

  while (1) {
    noInterrupts();
    int motorPulse = thrustPinData.pulseDuration;
    interrupts();

    bool direction = motorPulse > PWM_MID;
    digitalWrite(MOTOR_DIR_PIN, direction ? HIGH : LOW); // Set direction

    // Map PWM value to speed range (0 - 255)
    int speed;
    if (direction) {
      // CCW Rotation
      speed = map(motorPulse, PWM_MID, PWM_MAX, 0, 512);
    } else {
      // CW Rotation
      speed = map(motorPulse, PWM_MID, PWM_MIN, 0, 512);
    }
    speed = constrain(speed, 0, 512); // Ensure speed stays within bounds

    // MOTOR_DEADBAND
    if(motorPulse > (PWM_MID - MOTOR_DEADBAND) && motorPulse < (PWM_MID + MOTOR_DEADBAND)){
      speed = 0;
    }
    //Serial.printf("Pulse: %d\n", motorPulse);

    ledcWrite(MOTOR_PWM_PIN, speed);
    // sampling with 50Hz which correlates with pwm frequency of input signal 
    vTaskDelay(pdMS_TO_TICKS(100));//20));
  }
}
#elif defined(DRIVER_BTS7960)

void thrustControlTask(void *pvParameters) {
  pinMode(MOTOR_EN_PIN, OUTPUT);
  pinMode(MOTOR_PWM1_PIN, OUTPUT);
  pinMode(MOTOR_PWM2_PIN, OUTPUT);
  
  digitalWrite(MOTOR_EN_PIN,HIGH);

  ledcAttachChannel(MOTOR_PWM1_PIN, 20000, 9, 0);
  ledcAttachChannel(MOTOR_PWM2_PIN, 20000, 9, 1);
  ledcWrite(MOTOR_PWM2_PIN, 0);
  
  thrustPinData.pulseDuration = 1500;

  while (1) {
    noInterrupts();
    int motorPulse = thrustPinData.pulseDuration;
    interrupts();

    bool direction = motorPulse > PWM_MID;

    // Map PWM value to speed range (0 - 255)
    int speed;
    if (motorPulse > (PWM_MID + MOTOR_DEADBAND)) {
      // CCW Rotation
      ledcWrite(MOTOR_PWM1_PIN, 0);
      speed = map(motorPulse, PWM_MID, PWM_MAX, 0, 512);
      speed = constrain(speed, 0, 512); // Ensure speed stays within bounds
      ledcWrite(MOTOR_PWM2_PIN, speed);
    } else if(motorPulse < (PWM_MID - MOTOR_DEADBAND)) {
      // CW Rotation 
      ledcWrite(MOTOR_PWM2_PIN, 0);
      speed = map(motorPulse, PWM_MID, PWM_MIN, 0, 512);
      speed = constrain(speed, 0, 512); // Ensure speed stays within bounds
      ledcWrite(MOTOR_PWM1_PIN, speed);
    }
    else {
      // Within deadband, stop the motor
        ledcWrite(MOTOR_PWM1_PIN, 0);
        ledcWrite(MOTOR_PWM2_PIN, 0);
    }
    // sampling with 50Hz which correlates with pwm frequency of input signal 
    vTaskDelay(pdMS_TO_TICKS(50)); //20
  }
    
}
#endif

// Task function to toggle the LED
void toggleLED(void *parameter) {
  pinMode(ledPin, OUTPUT); // Set the LED pin as an output

  while (1) {
    digitalWrite(ledPin, HIGH); // Toggle the LED
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 500 milliseconds
    digitalWrite(ledPin, LOW); // Toggle the LED
    vTaskDelay(pdMS_TO_TICKS(200)); // Wait for 500 milliseconds
  }
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("\nstart...");

  // init interrupts for pwm input
  pinMode(thrustPinData.pin, INPUT);
  pinMode(stepperPinData.pin, INPUT);
  attachInterruptArg(digitalPinToInterrupt(thrustPinData.pin), handlePwmInterrupt, const_cast<void*>(reinterpret_cast<const volatile void*>(&thrustPinData)), CHANGE);
  attachInterruptArg(digitalPinToInterrupt(stepperPinData.pin), handlePwmInterrupt, const_cast<void*>(reinterpret_cast<const volatile void*>(&stepperPinData)), CHANGE);

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
  xTaskCreate(toggleLED, "Toggle LED", 1024, NULL, 1, NULL);
 
  // in order to achieve fast updates stepper control task runs at priority 0 with idle task
  // using taskYield in tasks with higher priority would starve the idle task and trigger the watchdog timeout
  xTaskCreate(stepperControlTask, "Control Stepper", 4096, NULL, 0, NULL);
  xTaskCreate(thrustControlTask, "Control Thrust Motor", 4096, NULL, 1, NULL);
  //xTaskCreate(readAnalogTask, "PPM generator", 4096, NULL, 1, NULL);
  //xTaskCreate(readPwmTask, "Read PWM", 1024, NULL, 1, NULL);
#ifdef UART_STEPPER_TEST_MODE
  xTaskCreate(readSerialTask, "Read PWM", 1024, NULL, 1, NULL);
#endif

  btHandler.init();  // Initialize Bluetooth
  // set bluetooth callbacks
  btHandler.setOnWriteCallback(onBluetoothWrite);
  btHandler.setOnConnectCallback(onBluetoothConnect);
  btHandler.setOnDisconnectCallback(onBluetoothDisconnect);

#ifdef USE_SBUS
  sbusTx.Begin();
#else
  ppm.init();
#endif
}

void loop() {
  // if this line is being hit something seriously went wrong
}

///////////////// CALLBACKS /////////////////

void onBluetoothWrite(const uint8_t* data, size_t length) {
    //Serial.printf("Received %d bytes via BLE\n", length);
    if(length == 8){
      int channel5 = data[0] << 8 | data [1];
      int channel1 = data[2] << 8 | data [3]; // roll
      int channel3 = data[4] << 8 | data [5]; // gier
      int channel4 = data[6] << 8 | data [7]; // arm
      int pulses[TX_CHANNELS] = {channel1,1500,channel3,channel4,channel5};
      //Serial.printf("1: %d\t3: %d\t4: %d\t5: %d\n", channel1, channel3,channel4, channel5);
      rcTransmit((const int *) pulses);
      //ppm.updateChannelPulses(pulses);
    }
}

void onBluetoothConnect() {
    Serial.println("BLE device connected");
    stepper.enableOutputs();
}

void onBluetoothDisconnect() {
    Serial.println("BLE device disconnected");
    const int ppmPulses[TX_CHANNELS] = {800, 800, 800, 800, 800};
    rcTransmit(ppmPulses);
    //ppm.updateChannelPulses(ppmPulses);
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

void rcTransmit(const int* channels){
#ifdef USE_SBUS
  for(int i = 0; i < TX_CHANNELS; ++i){
    sbusData.ch[i] = map(channels[i], 800, 2200, 0, 2047);
  }
  sbusTx.Write();
#else
  ppm.updateChannelPulses(channels);
#endif
}

#ifdef UART_STEPPER_TEST_MODE
// uart test mode
void readSerialTask(void *pvParameters) {
    static char serialBuffer[8]; // Adjusted buffer size to 8
    int index = 0;

    while (1) {
        if (Serial.available() > 0) {
            char receivedChar = Serial.read();
            
            // Check for newline character, which indicates the end of the input
            if (receivedChar == '\n' || receivedChar == '\r') {
                serialBuffer[index] = '\0'; // Null-terminate the string
                index = 0; // Reset index for the next message
                
                // Convert the received string to an integer
                stepperPinData.pulseDuration = atoi(serialBuffer);
                stepperPinData.pulseDuration = constrain(stepperPinData.pulseDuration, 1100, 1900);
            } else {
                // Add received character to buffer if it's not a newline
                // Ensure there's space for the null terminator
                if (index < sizeof(serialBuffer) - 1) {
                    serialBuffer[index++] = receivedChar;
                }
            }
        }
        
        // Add a small delay to prevent the task from using all CPU time
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
#endif

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