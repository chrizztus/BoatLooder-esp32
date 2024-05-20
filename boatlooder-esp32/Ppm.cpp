#include "Ppm.h"

// Constants
#define TIMER_NUMBER 0
#define PRESCALER 80
#define FRAME_LENGTH 22500
#define PULSE_HIGH_START 2000
#define PULSE_LOW 300

void Ppm::init() {
    pinMode(_ppmPin, OUTPUT);


    channelPulses = new int[_ppmChannels];
    for (int i = 0; i < _ppmChannels; i++) {
        channelPulses[i] = 800;  // Default pulse width
    }

    // init timer
    timer = timerBegin(1000000);
    //timer = timerBegin(TIMER_NUMBER, PRESCALER, true);
    timerAttachInterruptArg(timer, &Ppm::onPpmTimerInterrupt, this);
    timerAlarm(timer, FRAME_LENGTH, true, 0);
    //timerAlarmEnable(timer);
}

void Ppm::updateChannelPulses(const int* pulses) {
    for (int i = 0; i < _ppmChannels; i++) {
        channelPulses[i] = pulses[i];
    }
}

void IRAM_ATTR Ppm::onPpmTimerInterrupt(void* arg) {
    Ppm* ppm = static_cast<Ppm*>(arg);
    ppm->handlePpmInterrupt();
}

void Ppm::handlePpmInterrupt() {
    static int pulseIndex = 0;
    static unsigned long startTime = 0;
    static bool isHigh = true;
    if (pulseIndex == 0 && isHigh) {
        digitalWrite(_ppmPin, HIGH);
        startTime = micros();
        timerAlarm(timer, PULSE_HIGH_START, true, 0);
        isHigh = false;
    } else {
        if (isHigh) {
            int pulseDuration = (pulseIndex < _ppmChannels) ? channelPulses[pulseIndex - 1] - PULSE_LOW : channelPulses[pulseIndex - 1];
            digitalWrite(_ppmPin, HIGH);
            timerAlarm(timer, pulseDuration, true, 0);
            isHigh = false;
        } else {
            digitalWrite(_ppmPin, LOW);
            timerAlarm(timer, PULSE_LOW, true, 0);
            isHigh = true;

            if (pulseIndex < _ppmChannels) {
                pulseIndex++;
            } else {
                digitalWrite(_ppmPin, HIGH);
                unsigned long currentTime = micros();
                unsigned long elapsed = currentTime - startTime;
                unsigned long pauseTime = FRAME_LENGTH - elapsed;
                timerAlarm(timer, pauseTime, true, 0);
                pulseIndex = 0;
                isHigh = true;
            }
        }
    }
}
