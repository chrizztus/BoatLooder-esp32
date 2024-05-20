#ifndef PPM_H
#define PPM_H

#include <Arduino.h>

// #define PPM_PIN 14
// #define PPM_CHANNELS 5

class Ppm {
private:
    hw_timer_t* timer = NULL;
    int* channelPulses;
    int _ppmChannels;
    int _ppmPin;

    static void IRAM_ATTR onPpmTimerInterrupt(void* arg);

public:
    Ppm(int numChannels, int pin) : _ppmChannels(numChannels), _ppmPin(pin) {}
    void init();
    void updateChannelPulses(const int* pulses);
    void handlePpmInterrupt();
};

#endif