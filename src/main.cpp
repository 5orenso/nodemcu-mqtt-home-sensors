#include <Arduino.h>
#include <ESP8266WiFi.h>

#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <MqttUtil.h>
#include <PubSubClient.h>
#include <Bme280Sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <GenericAnalogSensor.h>
#include <PirSensor.h>
#include <Co2SensorMHZ19.h>

const char* PACKAGE_NAME = "nodemcu_mqtt_home_sensors";

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
#define BME280_SDA D2
#define BME280_SCL D1
#define ONE_WIRE_BUS 2  // DS18B20 on arduino pin2 corresponds to D4 on physical board
#define FLAME_SENSOR_PIN A0
#define LIGHT_SENSOR_PIN A0
#define GAS_MQ2_SENSOR_PIN A0
#define MOTION_SENSOR_PIN D5
#define CO2_MH_Z19_RX D7
#define CO2_MH_Z19_TX D6

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

const char *mqtt_server = MQTT_SERVER;
const int   mqtt_port = MQTT_PORT;

const char* outTopic = MQTT_OUT_TOPIC;
const char* inTopic = MQTT_IN_TOPIC;

WiFiEventHandler gotIpEventHandler, disconnectedEventHandler;
bool wifiConnected = false;
long wifiDisconnectedPeriode, wifiDisconnectedPeriodeStart;

WiFiClient espClient;
PubSubClient client(espClient);
long lastRun = millis();
int nodemcuChipId = ESP.getChipId(); // returns the ESP8266 chip ID as a 32-bit integer.

MqttUtil mqttUtil = MqttUtil(nodemcuChipId, PACKAGE_NAME, ssid, inTopic, outTopic, false);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);
Bme280Sensor bme280 = Bme280Sensor(BME280_SDA, BME280_SCL, 20, false);
GenericAnalogSensor flame = GenericAnalogSensor(FLAME_SENSOR_PIN, 20, false);
GenericAnalogSensor light = GenericAnalogSensor(LIGHT_SENSOR_PIN, 20, false);
GenericAnalogSensor gasMq2 = GenericAnalogSensor(GAS_MQ2_SENSOR_PIN, 20, false);
PirSensor motion = PirSensor(MOTION_SENSOR_PIN, 2, false, false);
Co2SensorMHZ19 co2 = Co2SensorMHZ19(CO2_MH_Z19_RX, CO2_MH_Z19_TX, 20, false);

void setupWifi() {
    delay(10);
    Serial.print("Connecting to "); Serial.println(ssid);

    // OTA wifi setting
    WiFi.mode(WIFI_STA);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print("."); Serial.print(ssid);
        delay(500);
    }
    randomSeed(micros());
    Serial.println(""); Serial.print("WiFi connected with IP: "); Serial.println(WiFi.localIP());
}

void setupOTA() {
    // OTA start
    // Port defaults to 8266
    // ArduinoOTA.setPort(8266);

    // Hostname defaults to esp8266-[ChipID]
    // ArduinoOTA.setHostname("myesp8266");

    // No authentication by default
    // ArduinoOTA.setPassword("admin");

    // Password can be set with it's md5 value as well
    // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
    // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else { // U_SPIFFS
            type = "filesystem";
        }
        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();
    // OTA end
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

void reconnectMqtt(uint32 ipAddress, long wifiDisconnectedPeriode) {
    while (!client.connected()) {
        String clientId = "ESP8266Client-";
        clientId += String(random(0xffff), HEX);
        Serial.print("Attempting MQTT connection: ["); Serial.print(clientId); Serial.print("] : ");
        if (client.connect(clientId.c_str())) {
            Serial.println("Connected to MQTT!");
            client.subscribe(inTopic);
            mqttUtil.sendControllerInfo(client, ipAddress, wifiDisconnectedPeriode);
        } else {
            Serial.print("failed, rc="); Serial.print(client.state()); Serial.println(" try again in 5 seconds...");
            delay(5000);
        }
    }
}

void setup(void) {
    Serial.begin(115200);
    gotIpEventHandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP& event) {
        wifiConnected = true;
        wifiDisconnectedPeriode = millis() - wifiDisconnectedPeriodeStart;
        Serial.print("Station connected, IP: "); Serial.println(WiFi.localIP());
    });
    disconnectedEventHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected& event) {
        wifiConnected = false;
        wifiDisconnectedPeriodeStart = millis();
        Serial.println("Station disconnected...");
    });
    setupWifi();
    setupOTA();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);

    if (BME280_SENSOR) {
        bme280.begin();
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
    delay(0); // Allow internal stuff to be executed.
    if (!wifiConnected) {
        delay(1000);
        return;
    }
    if (!client.connected()) {
        reconnectMqtt(WiFi.localIP(), wifiDisconnectedPeriode);
    }
    client.loop();

    // Listen for OTA requests
    ArduinoOTA.handle();

    // Stuff to do as soon as possible and as often as possible.
    if (BME280_SENSOR) {
        bme280.sampleValue();
    }
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
            mqttUtil.publishKeyValueInt(client, "motion", motionStateChange);
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
            float temperature = bme280.readValueTemperature();
            float pressure = bme280.readValuePressure();
            float humidity = bme280.readValueHumidity();
            mqttUtil.publishKeyValueFloat(client, "temperature", temperature, "pressure", pressure, "humidity", humidity, 4, 1);
        }
        if (DALLAS_TEMPERATURE_SENSOR) {
            // NOTE: Reading DallasTemperature sensor. This is kind of slow.
            //       Takes about 1 second to get something from this sensor.
            DS18B20.requestTemperatures();
            float waterTemperature = DS18B20.getTempCByIndex(0);
            // publishMqttMessage(waterTemperature, "waterTemp", 3, 1);
            mqttUtil.publishKeyValueFloat(client, "waterTemp", waterTemperature, 3, 1);
        }
        if (FLAME_SENSOR) {
            float valueFlame = flame.readValue();
            // publishMqttMessage(valueFlame, "flame", 3, 1);
            mqttUtil.publishKeyValueFloat(client, "flame", valueFlame, 3, 1);
        }
        if (LIGHT_SENSOR) {
            float valueLight = light.readValue();
            // publishMqttMessage(valueLight, "light", 3, 1);
            mqttUtil.publishKeyValueFloat(client, "light", valueLight, 3, 1);
        }
        if (GAS_MQ2_SENSOR) {
            float valueGasMq2 = gasMq2.readValue();
            if (valueGasMq2 > 0) {
                // publishMqttMessage(valueGasMq2, "GasMq2", 3, 1);
                mqttUtil.publishKeyValueFloat(client, "GasMq2", valueGasMq2, 3, 1);
            }
        }
        if (CO2_SENSOR) {
            float valueCo2 = co2.readValue();
            if (valueCo2 > 0) {
                // publishMqttMessage(valueCo2, "co2", 4, 1);
                mqttUtil.publishKeyValueFloat(client, "co2", valueCo2, 4, 1);
            }
        }

        if (DEEP_SLEEP) {
            Serial.print("Entering deep sleep mode for "); Serial.print(SLEEP_DELAY_IN_SECONDS); Serial.println(" seconds...");
            ESP.deepSleep(SLEEP_DELAY_IN_SECONDS * 1000000, WAKE_RF_DEFAULT);
        }
    }
}
