#include <Arduino.h>
#include <ESP8266WiFi.h>

#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

// #include <ESP8266mDNS.h>
// #include <WiFiUdp.h>

#include <MqttUtil.h>
#include <PubSubClient.h>
#include <Bme280Sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <GenericAnalogSensor.h>
#include <PirSensor.h>
// #include <Co2SensorMHZ19.h>

const char* PACKAGE_NAME = "nodemcu_mqtt_home_sensors";
const char* FW_NAME = NAME;
const int FW_VERSION = VERSION;
const char* FW_URL_BASE = FOTA_URL;

// #define DEBUG DEBUG
// #define VERBOSE VERBOSE
// #define DEEP_SLEEP DEEP_SLEEP

// #define BME280_SENSOR true
// #define DALLAS_TEMPERATURE_SENSOR false
// #define FLAME_SENSOR false
// #define LIGHT_SENSOR false
// #define GAS_MQ2_SENSOR false
// #define GAS_MQ3_SENSOR false
// #define MOTION_SENSOR false
// #define CO2_SENSOR false
// #define DSM501A_SENSOR false

// For deepsleep you shguld use a low PUBLISH_INTERVAL. Ie 5 sec or something.
// #define PUBLISH_INTERVAL 2
// #define SLEEP_DELAY_IN_SECONDS 300

#define FIRMWARE_CHECK_INTERVAL_IN_SEC 600

#define WIFI_CONNECT_ATTEMPTS 100
#define BME280_SDA D2
#define BME280_SCL D1
#define ONE_WIRE_BUS 2  // DS18B20 on arduino pin2 corresponds to D4 on physical board
#define FLAME_SENSOR_PIN A0
#define LIGHT_SENSOR_PIN A0
#define GAS_MQ2_SENSOR_PIN A0
#define GAS_MQ3_SENSOR_PIN A0
#define MOISTURE_SENSOR_PIN A0
#define MOTION_SENSOR_PIN D5
#define CO2_MH_Z19_RX D7
#define CO2_MH_Z19_TX D6
#define DSM501A_SENSOR_PIN D8

#if READ_VOLTAGE
    // Calibrate voltage function
    ADC_MODE(ADC_VCC); //vcc read
#endif


const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

const char *mqtt_server = MQTT_SERVER;
const int   mqtt_port = MQTT_PORT;

const char* outTopic = MQTT_OUT_TOPIC;
const char* inTopic = MQTT_IN_TOPIC;

WiFiEventHandler gotIpEventHandler, disconnectedEventHandler;
bool wifiConnected = false;
long wifiDisconnectedPeriode, wifiDisconnectedPeriodeStart;

WiFiClient mqttWifiClient;
WiFiClient firmwareWifiClient;

PubSubClient client(mqttWifiClient);

long startUpTimeEsp = millis();
long lastRun = millis();
long lastFirmwareCheck = millis();

int nodemcuChipId = ESP.getChipId(); // returns the ESP8266 chip ID as a 32-bit integer.
long lowpulseoccupancy = 0;

float voltage = ESP.getVcc();

long startUpTimeWifi = 0;
long connectTimeWifi = 0;
long startUpTimeMqtt = 0;
long connectTimeMqtt = 0;

MqttUtil mqttUtil = MqttUtil(nodemcuChipId, PACKAGE_NAME, ssid, inTopic, outTopic, false);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);
Bme280Sensor bme280 = Bme280Sensor(BME280_SDA, BME280_SCL, 20, false);
GenericAnalogSensor flame = GenericAnalogSensor(FLAME_SENSOR_PIN, 20, false);
GenericAnalogSensor light = GenericAnalogSensor(LIGHT_SENSOR_PIN, 20, false);
GenericAnalogSensor gasMq2 = GenericAnalogSensor(GAS_MQ2_SENSOR_PIN, 20, false);
GenericAnalogSensor gasMq3 = GenericAnalogSensor(GAS_MQ3_SENSOR_PIN, 20, false);
GenericAnalogSensor moisture = GenericAnalogSensor(MOISTURE_SENSOR_PIN, 20, false);
PirSensor motion = PirSensor(MOTION_SENSOR_PIN, 2, false, false);
// Co2SensorMHZ19 co2 = Co2SensorMHZ19(CO2_MH_Z19_RX, CO2_MH_Z19_TX, 20, false);

void checkForUpdates() {
    String fwURL = String(FW_URL_BASE);

    String fwVersionURL = fwURL;
    fwVersionURL.concat("version/"); fwVersionURL.concat(nodemcuChipId);
    fwVersionURL.concat("?v="); fwVersionURL.concat(FW_VERSION);
    fwVersionURL.concat("&name="); fwVersionURL.concat(FW_NAME);
    fwVersionURL.concat("&package="); fwVersionURL.concat(PACKAGE_NAME);
    fwVersionURL.concat("&ds="); fwVersionURL.concat(DEEP_SLEEP);
    fwVersionURL.concat("&bme280="); fwVersionURL.concat(BME280_SENSOR);
    fwVersionURL.concat("&dallas="); fwVersionURL.concat(DALLAS_TEMPERATURE_SENSOR);
    fwVersionURL.concat("&flame="); fwVersionURL.concat(FLAME_SENSOR);
    fwVersionURL.concat("&light="); fwVersionURL.concat(LIGHT_SENSOR);
    fwVersionURL.concat("&mq2="); fwVersionURL.concat(GAS_MQ2_SENSOR);
    fwVersionURL.concat("&mq3="); fwVersionURL.concat(GAS_MQ3_SENSOR);
    fwVersionURL.concat("&moisture="); fwVersionURL.concat(MOISTURE_SENSOR);
    fwVersionURL.concat("&motion="); fwVersionURL.concat(MOTION_SENSOR);
    fwVersionURL.concat("&co2="); fwVersionURL.concat(CO2_SENSOR);
    fwVersionURL.concat("&dsm501a="); fwVersionURL.concat(DSM501A_SENSOR);
    fwVersionURL.concat("&pi="); fwVersionURL.concat(PUBLISH_INTERVAL);
    fwVersionURL.concat("&sp="); fwVersionURL.concat(SLEEP_DELAY_IN_SECONDS);
    fwVersionURL.concat("&wifi="); fwVersionURL.concat(WIFI_SSID);
    fwVersionURL.concat("&mqs="); fwVersionURL.concat(MQTT_SERVER);
    fwVersionURL.concat("&mqp="); fwVersionURL.concat(MQTT_PORT);
    fwVersionURL.concat("&mqout="); fwVersionURL.concat(MQTT_OUT_TOPIC);
    fwVersionURL.concat("&mqin="); fwVersionURL.concat(MQTT_IN_TOPIC);
    fwVersionURL.concat("&time="); fwVersionURL.concat(millis());

    Serial.println("Checking for firmware updates.");
    Serial.print("ChipId: ");
    Serial.println(nodemcuChipId);
    Serial.print("Firmware version URL: ");
    Serial.println(fwVersionURL);

    HTTPClient httpClient;
    if (httpClient.begin(firmwareWifiClient, fwVersionURL)) {
        int httpCode = httpClient.GET();
        if (httpCode == 200 || httpCode == 304) {
            String newFWVersion = httpClient.getString();

            Serial.print("Current firmware version: "); Serial.println(FW_VERSION);
            Serial.print("Available firmware version: "); Serial.println(newFWVersion);

            int newVersion = newFWVersion.toInt();

            if (newVersion > FW_VERSION) {
                Serial.println("Preparing to update");

                String fwImageURL = fwURL;
                fwImageURL.concat("firmware/"); fwImageURL.concat(nodemcuChipId);
                Serial.print("Firmware binary URL: "); Serial.println(fwImageURL);
                t_httpUpdate_return ret = ESPhttpUpdate.update(firmwareWifiClient, fwImageURL);

                switch(ret) {
                    case HTTP_UPDATE_FAILED:
                        Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
                        break;
                    case HTTP_UPDATE_NO_UPDATES:
                        Serial.println("HTTP_UPDATE_NO_UPDATES");
                        break;
                    case HTTP_UPDATE_OK:
                        Serial.println("HTTP_UPDATE_OK");
                        break;
                }
            } else {
                Serial.println("Already on latest version");
            }
        } else {
            Serial.print("Firmware version check failed, got HTTP response code ");
            Serial.println(httpCode);
        }
        httpClient.end();
    } else {
        Serial.println("HTTP Error: Unable to connect");
    }
    delay(10);
}

void setupWifi() {
    // Static IP
    // IPAddress ip( 192, 168, 0, 1 );
    // IPAddress gateway( 192, 168, 0, 254 );
    // IPAddress subnet( 255, 255, 255, 0 );
    startUpTimeWifi = millis();
    
    // WiFi.forceSleepEnd();
    WiFi.forceSleepWake();
    delay(10);

    Serial.print("WiFi Connecting to "); Serial.println(ssid);

    // Disable the WiFi persistence.  The ESP8266 will not load and save WiFi settings in the flash memory.
    WiFi.persistent(false);
    delay(10);

    // Bring up the WiFi connection
    WiFi.mode(WIFI_STA);
    delay(10);

    // Static IP
    // WiFi.config( ip, gateway, subnet );

    int wifi_attempts = 0;
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
        wifi_attempts++;
        if (wifi_attempts > WIFI_CONNECT_ATTEMPTS) {
            Serial.println("Can't connect to the wifi, going to deep sleep");
            ESP.deepSleep(SLEEP_DELAY_IN_SECONDS * 1000000, WAKE_RF_DISABLED);
            delay(10);
        }
    }
    randomSeed(micros());
    Serial.println(""); Serial.print("WiFi connected with IP: "); Serial.println(WiFi.localIP());
    connectTimeWifi = millis();
}

void shutdownWifi() {
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();

    // One of the issues at this point though, is that the WiFi can take a finite, but variable 
    // time to shut down after the final forceSleepBegin() call, so at this point I’ve just 
    // added a timeout loop, checking the status of the connection until it goes down (returning 
    // without an error) or times out (returning with an error):-
    int wifi_attempts = 0;
    while ((WiFi.status() == WL_CONNECTED) && (wifi_attempts++ < WIFI_CONNECT_ATTEMPTS)) {
        delay(100);
        Serial.print(".");
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(""); Serial.print("WiFi is now offline.");
    } else {
        Serial.println(""); Serial.print("Not able to shutdown WiFi...");
    }
    delay(100);
}

void callback(char* topic, byte* payload, int length) {
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
    startUpTimeMqtt = millis();
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
            delay(500);
        }
    }
    connectTimeMqtt = millis();
}

void setup(void) {
    shutdownWifi();

    Serial.begin(115200);
    gotIpEventHandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP& event) {
        wifiConnected = true;
        wifiDisconnectedPeriode = millis() - wifiDisconnectedPeriodeStart;
        Serial.print("WiFi connected, IP: "); Serial.println(WiFi.localIP());
    });
    disconnectedEventHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected& event) {
        wifiConnected = false;
        wifiDisconnectedPeriodeStart = millis();
        Serial.println("WiFi disconnected...");
    });
    // setupWifi();
    // client.setServer(mqtt_server, mqtt_port);
    // client.setCallback(callback);

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
    if (GAS_MQ3_SENSOR) {
        gasMq3.begin();
    }
    if (MOISTURE_SENSOR) {
        moisture.begin();
    }
    if (MOTION_SENSOR) {
        motion.begin();
    }
    // if (CO2_SENSOR) {
    //     co2.begin();
    // }
    if (DSM501A_SENSOR) {
        // co2.begin();
    }
}

void loop() {
    delay(10); // Allow internal stuff to be executed.
  
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
    if (GAS_MQ3_SENSOR) {
        gasMq3.sampleValue();
    }
    if (MOISTURE_SENSOR) {
        moisture.sampleValue();
    }
    if (MOTION_SENSOR) {
        int motionStateChange = motion.sampleValue();
        if (motionStateChange >= 0) {
            mqttUtil.publishKeyValueInt(client, "motion", motionStateChange);
        }
    }
    // if (CO2_SENSOR) {
    //     co2.sampleValue();
    // }
    if (DSM501A_SENSOR) {
        long duration = pulseIn(DSM501A_SENSOR_PIN, LOW);
        lowpulseoccupancy += duration;
    }

    // Stuff to do at given time intervals.
    long now = millis();
    if (now - lastRun > (PUBLISH_INTERVAL * 1000)) {
        if (!wifiConnected) {
            setupWifi();
            delay(10);
        }
        if (!client.connected()) {
            client.setServer(mqtt_server, mqtt_port);
            client.setCallback(callback);
            reconnectMqtt(WiFi.localIP(), wifiDisconnectedPeriode);
        }
        client.loop();

        if (READ_VOLTAGE) {
            // 1. The range of the operating voltage of ESP8266 is 1.8V-3.6V
            // 2. getVcc function (system_get_vdd33) is only available when TOUT pin17 is suspended (floating),
            //    this function measures the power voltage of VDD3P3 pin 3 and 4 (in the ESP8266 chip).
            // 3. RF must be enabled.
            mqttUtil.publishKeyValueFloat(client, "voltage", voltage / 1000, 3, 2);
        }
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
        if (GAS_MQ3_SENSOR) {
            float valueGasMq3 = gasMq3.readValue();
            if (valueGasMq3 > 0) {
                // publishMqttMessage(valueGasMq2, "GasMq2", 3, 1);
                mqttUtil.publishKeyValueFloat(client, "GasMq3", valueGasMq3, 3, 1);
            }
        }
        if (MOISTURE_SENSOR) {
            float valueMoisture = moisture.readValue();
            if (valueMoisture > 0) {
                // publishMqttMessage(valueGasMq2, "GasMq2", 3, 1);
                mqttUtil.publishKeyValueFloat(client, "Moisture", valueMoisture, 3, 1);
            }
        }
        // if (CO2_SENSOR) {
        //     float valueCo2 = co2.readValue();
        //     if (valueCo2 > 0) {
        //         // publishMqttMessage(valueCo2, "co2", 4, 1);
        //         mqttUtil.publishKeyValueFloat(client, "co2", valueCo2, 4, 1);
        //     }
        // }
        if (DSM501A_SENSOR) {
            float ratio = lowpulseoccupancy / ((PUBLISH_INTERVAL * 1000) * 10.0); // Converts to ratio of low time
            float concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62; // Calculates
            concentration = concentration * (3531.46); // Converts to Cubic meter
            double particleMass = ratio * 0.11667; // Calculated as linear interpolation of spec data, as 1.14/0.12. (table 8.2)

            // Serial.println("\n******** NEW MEASUREMENT ****************");
            // Serial.print("*Concentration = ");
            // Serial.print(concentration);
            // Serial.print(" pcs/m3");
            // Serial.println("\t*"); //Tab
            // Serial.print("*Particle mass = ");
            // Serial.print(particleMass);
            // Serial.print(" mg/m3");
            // Serial.println("\t\t\t*");
            // Serial.print("*Ratio = ");
            // Serial.print(ratio);
            // Serial.println("\t\t\t\t*");
            // Serial.println("*****************************************");

            if (concentration > 0) {
                // publishMqttMessage(valueCo2, "co2", 4, 1);
                mqttUtil.publishKeyValueFloat(client, "concentration", concentration, "particleMass", particleMass, "ratio", ratio, 7, 2);
            }

            lowpulseoccupancy = 0;  //Reset value
        }

        long finalTimeEsp = millis();
        mqttUtil.publishKeyValueFloat(client, "espTotalRunTime", finalTimeEsp - startUpTimeEsp, 5, 0);
        mqttUtil.publishKeyValueFloat(client, "wifiConnectTime", connectTimeWifi - startUpTimeWifi, 5, 0);
        mqttUtil.publishKeyValueFloat(client, "wifiOnlineTime", finalTimeEsp - connectTimeWifi, 5, 0);
        mqttUtil.publishKeyValueFloat(client, "mqttConnectTime", connectTimeMqtt - startUpTimeMqtt, 5, 0);
        mqttUtil.publishKeyValueInt(client, "FW_VERSION", FW_VERSION);

        lastRun = now;
        if (DEEP_SLEEP) {
            // Wait for mqtt stuff to finish before shutting down.
            delay(200);
            // Check for firmware updates before we go to sleep.
            checkForUpdates();

            mqttUtil.disconnect(client);
            delay(10);

            // Shutdown wifi and go to sleep:
            shutdownWifi();

            Serial.print("Entering deep sleep mode for "); Serial.print(SLEEP_DELAY_IN_SECONDS); Serial.println(" seconds...");
            // WAKE_RF_DISABLED to keep the WiFi radio disabled when we wake up
            ESP.deepSleep(SLEEP_DELAY_IN_SECONDS * 1000000, WAKE_RF_DISABLED);
            delay(100);
            Serial.print("YOU SHOULD NEVER SEE THIS TEXT! DeepSleep is not working... ");
        } else if (now - lastFirmwareCheck > (FIRMWARE_CHECK_INTERVAL_IN_SEC * 1000)) {
            Serial.println("No deepsleep, so we need to check for firmare updates now and then.");
            Serial.print(FIRMWARE_CHECK_INTERVAL_IN_SEC); Serial.println(" seconds between every check.");
            checkForUpdates();
            lastFirmwareCheck = now;
        }
    }
}
