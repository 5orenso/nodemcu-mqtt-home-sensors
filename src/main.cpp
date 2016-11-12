/*
 It connects to an MQTT server then:
  - publishes "hello world" to the topic "outTopic" every two seconds
  - subscribes to the topic "inTopic", printing out any messages
    it receives. NB - it assumes the received payloads are strings not binary
  - If the first character of the topic "inTopic" is an 1, switch ON the ESP Led,
    else switch it off
 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.
*/
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <BME280.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <FlameSensor.h>
#include <LightSensor.h>
#include <PirSensor.h>

#define DEBUG false
#define VERBOSE true

#define DEEP_SLEEP true

#define BME280_SENSOR true
#define DALLAS_TEMPERATURE_SENSOR false
#define FLAME_SENSOR false
#define LIGHT_SENSOR true
#define MOTION_SENSOR false

#define PUBLISH_INTERVAL 15
#define SLEEP_DELAY_IN_SECONDS 120
#define ONE_WIRE_BUS 2  // DS18B20 on arduino pin2 corresponds to D4 on physical board
#define FLAME_SENSOR_PIN A0
#define LIGHT_SENSOR_PIN A0
#define MOTION_SENSOR_PIN D6

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

const char* mqtt_server = MQTT_SERVER;
const int   mqtt_port = MQTT_PORT;

const char* outTopic = MQTT_OUT_TOPIC;
const char* inTopic = MQTT_IN_TOPIC;

WiFiClient espClient;
PubSubClient client(espClient);
long lastRun = millis();
char msg[150];
int chipId = ESP.getChipId();

BME280 bme280; // SDA = D2, SCL = D1

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);

FlameSensor flame = FlameSensor(FLAME_SENSOR_PIN, 20, false);
LightSensor light = LightSensor(LIGHT_SENSOR_PIN, 20, false);
PirSensor motion = PirSensor(MOTION_SENSOR_PIN, 2, false, false);

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

    // Switch on the LED if an 1 was received as first character
    if ((char)payload[0] == '1') {
        // it is acive low on the ESP-01)
    } else { }
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
            // client.publish(outTopic, "{ \"chipId\": chipId, \"ping\": \"hello world\" }");
            // ... and resubscribe
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

static void setupBME280Sensor(void) {
    if (!bme280.begin()) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
    }
}

void setup(void) {
    Serial.begin(115200);
    setupBME280Sensor(); // Setup BME280 sensor
    if (DALLAS_TEMPERATURE_SENSOR) {
        DS18B20.begin();  // Init DallasTemperature sensor
    }
    if (FLAME_SENSOR) {
        flame.begin();
    }
    if (LIGHT_SENSOR) {
        light.begin();
    }
    if (MOTION_SENSOR) {
        motion.begin();
    }
    setupWifi();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
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
    if (MOTION_SENSOR) {
        int motionStateChange = motion.sampleValue();
        if (motionStateChange >= 0) {
            if (DEBUG) {
                Serial.print(" --> motionStateChange: "); Serial.println(motionStateChange);
            }
            // dtostrf(temperature, 3, 2, temperatureValue); // first 2 is the width including the . (1.) and the 2nd 2 is the precision (.23)
            snprintf(msg, 150, "{ \"chipId\": %d, \"motion\": %d }", chipId, motionStateChange);
            if (VERBOSE) {
                Serial.print("Publish message: "); Serial.println(msg);
            }
            client.publish(outTopic, msg);
        }
    }

    // Stuff to do at given time intervals.
    long now = millis();
    if (now - lastRun > (PUBLISH_INTERVAL * 1000)) {
        lastRun = now;
        float value = 0.0;
        char temperatureValue[10];
        char pressureValue[10];
        char humidityValue[10];
        char valueFlameValue[10];
        char valueLightValue[10];
        char waterTemperatureValue[10];
        int valueMotion = 0;

        if (BME280_SENSOR) {
            // Reading BME280 sensor
            float temperature = bme280.ReadTemperature();
            dtostrf(temperature, 3, 2, temperatureValue); // first 2 is the width including the . (1.) and the 2nd 2 is the precision (.23)
            float pressure = bme280.ReadPressure() / 100;
            dtostrf(pressure, 4, 1, pressureValue);
            float humidity = bme280.ReadHumidity();
            dtostrf(humidity, 3, 0, humidityValue);
            if (DEBUG) {
                Serial.print(", Air temperature:"); Serial.print(temperatureValue); Serial.print(" C");
                Serial.print(", Air pressure:"); Serial.print(pressureValue); Serial.print(" hPa");
                Serial.print(", Air humidity:"); Serial.print(humidityValue); Serial.print(" %");
            }
            // Sending to MQTT
            snprintf (msg, 150, "{ \"chipId\": %d, \"temperature\": %s, \"pressure\": %s, \"humidity\": %s }",
                chipId, temperatureValue, pressureValue, humidityValue);
            if (VERBOSE) {
                Serial.print("Publish message: "); Serial.println(msg);
            }
            client.publish(outTopic, msg);
        }

        if (DALLAS_TEMPERATURE_SENSOR) {
            // Reading DallasTemperature sensor. This is kind of slow.
            // Takes about 1 second to get something from this sensor.
            DS18B20.requestTemperatures();
            float waterTemperature = DS18B20.getTempCByIndex(0);
            dtostrf(waterTemperature, 3, 1, waterTemperatureValue);
            if (DEBUG) {
                Serial.print(", Water temp:"); Serial.print(waterTemperatureValue); Serial.print(" C");
            }
        } else {
            dtostrf(0, 3, 1, waterTemperatureValue);
        }
        if (FLAME_SENSOR) {
            // Reading flame sensor.
            float valueFlame = flame.readValue();
            dtostrf(valueFlame, 3, 1, valueFlameValue);
            if (DEBUG) {
                Serial.print(", Flame: "); Serial.print(valueFlameValue);
            }
        } else {
            dtostrf(0, 3, 1, valueFlameValue);
        }
        if (MOTION_SENSOR) {
            // Reading PIR (motion) sensor
            valueMotion = round(motion.readValue());
            if (DEBUG) {
                Serial.print(", Motion: "); Serial.print(valueMotion);
            }
        }
        // Send to MQTT
        // TODO: Maybe split up the different sensors into own sendings?
        if ((DALLAS_TEMPERATURE_SENSOR || FLAME_SENSOR || MOTION_SENSOR)) {
            // Sending to MQTT
            snprintf (msg, 150, "{ \"chipId\": %d, \"waterTemp\": %s, \"flame\": %s, \"motion\": %d }",
                chipId, waterTemperatureValue, valueFlameValue, valueMotion);
            if (VERBOSE) {
                Serial.print("Publish message: "); Serial.println(msg);
            }
            client.publish(outTopic, msg);
        }

        // Other setup
        if (LIGHT_SENSOR) {
            // Reading flame sensor.
            float valueLight = light.readValue();
            dtostrf(valueLight, 3, 1, valueLightValue);
            if (DEBUG) {
                Serial.print(", Light: "); Serial.print(valueLightValue);
            }
            // Sending to MQTT
            snprintf (msg, 150, "{ \"chipId\": %d, \"light\": %s }",
                chipId, valueLightValue);
            if (VERBOSE) {
                Serial.print("Publish message: "); Serial.println(msg);
            }
            client.publish(outTopic, msg);
        }

        if (DEEP_SLEEP) {
            Serial.print("Entering deep sleep mode for "); Serial.print(SLEEP_DELAY_IN_SECONDS); Serial.println(" seconds...");
            ESP.deepSleep(SLEEP_DELAY_IN_SECONDS * 1000000, WAKE_RF_DEFAULT);
        }
    }
}
