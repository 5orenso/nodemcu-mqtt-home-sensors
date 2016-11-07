#ifndef LIGHT_SENSOR_H
#define LIGHT_SENSOR_H

#include <Arduino.h>

class LightSensor {
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
        LightSensor(uint8_t sensorPin, int samplesCount, bool debugSetting);
        bool begin();
        float sampleValue();
        float readValue();
};

#endif //LIGHT_SENSOR_H