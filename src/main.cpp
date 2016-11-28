#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <BME280.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <GenericAnalogSensor.h>
#include <PirSensor.h>
#include <Co2SensorMHZ19.h>

#define DEBUG false
#define VERBOSE true
#define DEEP_SLEEP false

#define BME280_SENSOR true
#define DALLAS_TEMPERATURE_SENSOR false
#define FLAME_SENSOR false
#define LIGHT_SENSOR false
#define GAS_MQ2_SENSOR true
#define MOTION_SENSOR false
#define CO2_SENSOR true

#define PUBLISH_INTERVAL 15
#define SLEEP_DELAY_IN_SECONDS 60
#define ONE_WIRE_BUS 2  // DS18B20 on arduino pin2 corresponds to D4 on physical board
#define FLAME_SENSOR_PIN A0
#define LIGHT_SENSOR_PIN A0
#define GAS_MQ2_SENSOR_PIN A0
#define MOTION_SENSOR_PIN D5
#define MH_Z19_RX D7
#define MH_Z19_TX D8

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

const char *mqtt_server = MQTT_SERVER;
const int   mqtt_port = MQTT_PORT;

const char* outTopic = MQTT_OUT_TOPIC;
const char* inTopic = MQTT_IN_TOPIC;

WiFiClient espClient;
PubSubClient client(espClient);
long lastRun = millis();

int nodemcuChipId = ESP.getChipId(); // returns the ESP8266 chip ID as a 32-bit integer.
// ESP.getResetReason() returns String containing the last reset resaon in human readable format.
int nodemcuFreeHeapSize = ESP.getFreeHeap(); // returns the free heap size.
// Several APIs may be used to get flash chip info:
int nodemcuFlashChipId = ESP.getFlashChipId(); // returns the flash chip ID as a 32-bit integer.
int nodemcuFlashChipSize = ESP.getFlashChipSize(); // returns the flash chip size, in bytes, as seen by the SDK (may be less than actual size).
// int nodemcuFlashChipSpeed = ESP.getFlashChipSpeed(void); // returns the flash chip frequency, in Hz.
int nodemcuCycleCount = ESP.getCycleCount(); // returns the cpu instruction cycle count since start as an unsigned 32-bit. This is useful for accurate timing of very short actions like bit banging.
// WiFi.macAddress(mac) is for STA, WiFi.softAPmacAddress(mac) is for AP.
// int nodemcuIP; // = WiFi.localIP(); // is for STA, WiFi.softAPIP() is for AP.

BME280 bme280; // SDA = D2, SCL = D1
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);
GenericAnalogSensor flame = GenericAnalogSensor(FLAME_SENSOR_PIN, 20, false);
GenericAnalogSensor light = GenericAnalogSensor(LIGHT_SENSOR_PIN, 20, false);
GenericAnalogSensor gasMq2 = GenericAnalogSensor(GAS_MQ2_SENSOR_PIN, 20, false);
PirSensor motion = PirSensor(MOTION_SENSOR_PIN, 2, false, false);
Co2SensorMHZ19 co2 = Co2SensorMHZ19(MH_Z19_RX, MH_Z19_TX, 20, false);

void setupWifi() {
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print("."); Serial.print(ssid);
        delay(500);
    }
    randomSeed(micros());
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
    if ((char)payload[0] == '1') {
        // it is acive low on the ESP-01)
    } else {

    }
}

void reconnect() {
    // Loop until we're reconnected
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Create a random client ID
        String clientId = "ESP8266Client-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect
        if (client.connect(clientId.c_str())) {
            Serial.println("connected");
            // Once connected, publish an announcement...
            client.subscribe(inTopic);
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

void setupBME280Sensor(void) {
    if (!bme280.begin()) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
    }
}

bool publishMqttMessage(float value, char const *key, int length, int decimal) {
    char msg[150];
    char textValue[10];
    dtostrf(value, length, decimal, textValue); // first 2 is the width including the . (1.) and the 2nd 2 is the precision (.23)
    // snprintf(msg, 150, "{ \"chipId\": %d, \"freeHeapSize\": %d, \"flashChipSize\": %d, \"cycleCount\": %d, \"%s\": %s }", nodemcuChipId, nodemcuFreeHeapSize, nodemcuFlashChipSize, nodemcuCycleCount, key, textValue);
    snprintf(msg, 150, "{ \"chipId\": %d, \"flashChipId\": %d, \"%s\": %s }", nodemcuChipId, nodemcuFlashChipId, key, textValue);
    if (VERBOSE) {
        Serial.print("Publish message: "); Serial.println(msg);
    }
    client.publish(outTopic, msg);
    return true;
}

void setup(void) {
    Serial.begin(115200);
    setupWifi();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    if (BME280_SENSOR) {
        setupBME280Sensor(); // Setup BME280 sensor
    }
    if (DALLAS_TEMPERATURE_SENSOR) {
        DS18B20.begin();  // Init DallasTemperature sensor
    }
    if (FLAME_SENSOR) {
        flame.begin();
    }
    if (LIGHT_SENSOR) {
        light.begin();
    }
    if (GAS_MQ2_SENSOR) {
        gasMq2.begin();
    }
    if (MOTION_SENSOR) {
        motion.begin();
    }
    if (CO2_SENSOR) {
        co2.begin();
    }
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    // Stuff to do as soon as possible and as often as possible.
    if (FLAME_SENSOR) {
        flame.sampleValue();
    }
    if (LIGHT_SENSOR) {
        light.sampleValue();
    }
    if (GAS_MQ2_SENSOR) {
        gasMq2.sampleValue();
    }
    if (MOTION_SENSOR) {
        int motionStateChange = motion.sampleValue();
        if (motionStateChange >= 0) {
            publishMqttMessage(motionStateChange, "motion", 2, 0);
        }
    }
    if (CO2_SENSOR) {
        co2.sampleValue();
    }

    // Stuff to do at given time intervals.
    long now = millis();
    if (now - lastRun > (PUBLISH_INTERVAL * 1000)) {
        lastRun = now;

        if (BME280_SENSOR) {
            float temperature = bme280.ReadTemperature();
            publishMqttMessage(temperature, "temperature", 3, 2);
            float pressure = bme280.ReadPressure() / 100;
            publishMqttMessage(pressure, "pressure", 4, 1);
            float humidity = bme280.ReadHumidity();
            publishMqttMessage(humidity, "humidity", 3, 1);
        }
        if (DALLAS_TEMPERATURE_SENSOR) {
            // NOTE: Reading DallasTemperature sensor. This is kind of slow.
            //       Takes about 1 second to get something from this sensor.
            DS18B20.requestTemperatures();
            float waterTemperature = DS18B20.getTempCByIndex(0);
            publishMqttMessage(waterTemperature, "waterTemp", 3, 1);
        }
        if (FLAME_SENSOR) {
            float valueFlame = flame.readValue();
            publishMqttMessage(valueFlame, "flame", 3, 1);

        }
        if (LIGHT_SENSOR) {
            float valueLight = light.readValue();
            publishMqttMessage(valueLight, "light", 3, 1);
        }
        if (GAS_MQ2_SENSOR) {
            float valueGasMq2 = gasMq2.readValue();
            if (valueGasMq2 > 0) {
                publishMqttMessage(valueGasMq2, "GasMq2", 3, 1);
            }
        }
        if (CO2_SENSOR) {
            float valueCo2 = co2.readValue();
            if (valueCo2 > 0) {
                publishMqttMessage(valueCo2, "co2", 4, 1);
            }
        }

        if (DEEP_SLEEP) {
            Serial.print("Entering deep sleep mode for "); Serial.print(SLEEP_DELAY_IN_SECONDS); Serial.println(" seconds...");
            ESP.deepSleep(SLEEP_DELAY_IN_SECONDS * 1000000, WAKE_RF_DEFAULT);
        }
    }
}
