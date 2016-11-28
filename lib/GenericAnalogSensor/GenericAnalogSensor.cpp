#include <GenericAnalogSensor.h>
// #include <Math.h>
#if ARDUINO >= 100
#include "Arduino.h"
#else
extern "C" {
#include "WConstants.h"
}
#endif
// #include <Arduino.h>

GenericAnalogSensor::GenericAnalogSensor(uint8_t sensorPin, uint8_t samplesCount, bool debugSetting) {
    debug = debugSetting;
    sampleIndex = 0;
    lastSample = 0;
    samplesTotal = samplesCount;
    samples = new float[samplesCount];
    pin = sensorPin;
}

bool GenericAnalogSensor::begin() {
    pinMode(pin, INPUT);
    return true;
}

float GenericAnalogSensor::sampleValue() {
    long now = millis();
    // Don't sample more often than every 50 millisecond.
    if (now - lastSample > 50) {
        lastSample = now;
        samples[sampleIndex] = analogRead(pin);
        // if (debug) {
        //     Serial.print(" -->"); Serial.print(sampleIndex); Serial.print(": "); Serial.println(samples[sampleIndex]);
        // }
        sampleIndex++;
        if (sampleIndex >= samplesTotal) {
            sampleIndex = 0;
        }
        return samples[sampleIndex];
    }
}

float GenericAnalogSensor::readValue() {
    float average;
    average = 0;
    int valuesCount = 0;
    for (int i = 0; i < samplesTotal; i++) {
        // if (debug) {
        //     Serial.print("  -->"); Serial.print(i); Serial.print(": "); Serial.println(samples[i]);
        // }
        if (samples[i] > 0.00) {
            valuesCount++;
            average += samples[i];
        }
    }
    // if (debug) {
    //     Serial.print(" ==>"); Serial.println(average);
    // }
    average = (float)average / (float)valuesCount;

    return average;
}