#include <Bme280Sensor.h>
#include <Wire.h>
#include <BME280.h>
#include <Math.h>
#include <Arduino.h>

using namespace std;
 // SDA = D2, SCL = D1

Bme280Sensor::Bme280Sensor(uint8_t sdaPinInput, uint8_t sclPinInput, int samplesCount, bool debugSetting) {
    debug = debugSetting;
    sampleIndex = 0;
    lastSample = 0;
    samplesTotal = samplesCount;
    samplesTemperature = new float[samplesCount];
    samplesPressure = new float[samplesCount];
    samplesHumidity = new float[samplesCount];
    sdaPin = sdaPinInput;
    sclPin = sclPinInput;
}

bool Bme280Sensor::begin() {
    if (!bme280.begin()) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
    }
}

float Bme280Sensor::sampleValue() {
    long now = millis();
    // Don't sample more often than every 200 millisecond.
    if (now - lastSample > 200) {
        lastSample = now;

        float temperature = bme280.ReadTemperature();
        float pressure = bme280.ReadPressure() / 100;
        float humidity = bme280.ReadHumidity();

        samplesTemperature[sampleIndex] = temperature;
        samplesPressure[sampleIndex] = pressure;
        samplesHumidity[sampleIndex] = humidity;

        sampleIndex++;
        if (sampleIndex >= samplesTotal) {
            sampleIndex = 0;
        }
        return samplesTemperature[sampleIndex];
    }
}

float Bme280Sensor::readValueTemperature() {
    float average = 0;
    int valuesCount = 0;
    for (int i = 0; i < samplesTotal; i++) {
        if (samplesTemperature[i] > -50.00) {
            valuesCount++;
            average += samplesTemperature[i];
        }
    }
    average = (float)average / (float)valuesCount;
    return average;
}
float Bme280Sensor::readValuePressure() {
    float average = 0;
    int valuesCount = 0;
    for (int i = 0; i < samplesTotal; i++) {
        if (samplesPressure[i] > 0.00) {
            valuesCount++;
            average += samplesPressure[i];
        }
    }
    average = (float)average / (float)valuesCount;
    return average;
}
float Bme280Sensor::readValueHumidity() {
    float average = 0;
    int valuesCount = 0;
    for (int i = 0; i < samplesTotal; i++) {
        if (samplesHumidity[i] > 0.00) {
            valuesCount++;
            average += samplesHumidity[i];
        }
    }
    average = (float)average / (float)valuesCount;
    return average;
}
