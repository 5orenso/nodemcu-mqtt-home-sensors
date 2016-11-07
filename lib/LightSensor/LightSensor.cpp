#include <LightSensor.h>
#include <Math.h>
#include <Arduino.h>

using namespace std;

LightSensor::LightSensor(uint8_t sensorPin, int samplesCount, bool debugSetting) {
    debug = debugSetting;
    sampleIndex = 0;
    lastSample = 0;
    samplesTotal = samplesCount;
    samples = new float[samplesCount];
    pin = sensorPin;
}

bool LightSensor::begin() {
    pinMode(pin, INPUT);
}

float LightSensor::sampleValue() {
    long now = millis();
    // Don't sample more often than every 50 millisecond.
    if (now - lastSample > 50) {
        lastSample = now;
        samples[sampleIndex] = analogRead(pin);
        if (debug) {
            Serial.print(" -->"); Serial.print(sampleIndex); Serial.print(": "); Serial.println(samples[sampleIndex]);
        }
        sampleIndex++;
        if (sampleIndex >= samplesTotal) {
            sampleIndex = 0;
        }
        return samples[sampleIndex];
    }
}

float LightSensor::readValue() {
    float average;
    average = 0;
    int valuesCount = 0;
    for (int i = 0; i < samplesTotal; i++) {
        if (debug) {
            Serial.print("  -->"); Serial.print(i); Serial.print(": "); Serial.println(samples[i]);
        }
        if (samples[i] > 0.00) {
            valuesCount++;
            average += samples[i];
        }
    }
    if (debug) {
        Serial.print(" ==>"); Serial.println(average);
    }
    average = (float)average / (float)valuesCount;

    return average;
}