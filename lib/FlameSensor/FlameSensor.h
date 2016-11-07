#ifndef FLAME_SENSOR_H
#define FLAME_SENSOR_H

#include <Arduino.h>

class FlameSensor {
    bool
        debug;
    int
        sampleIndex,
        samplesTotal;
    float
        *samples;
    uint8_t
        pin;
    long
        lastSample;
    public:
        FlameSensor(uint8_t sensorPin, int samplesCount, bool debugSetting);
        bool begin();
        float sampleValue();
        float readValue();
};

#endif //FLAME_SENSOR_H