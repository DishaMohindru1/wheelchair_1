#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MLX90614.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <WiFiNINA.h>
#include <PubSubClient.h>

const int ECG_PIN = A0;
const int SDA_PIN = 2;
const int SCL_PIN = 3;
const int LED_PIN = 4;
const int SDA_PIN_PULSE = 14;
const int SCL_PIN_PULSE = 15;

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MAX30105 particleSensor;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setup() {
  Serial.begin(9600);
  mlx.begin();
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire1.begin(SDA_PIN_PULSE, SCL_PIN_PULSE);
  particleSensor.begin(Wire1, I2C_SPEED_FAST);
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);

  WiFi.begin("your_wifi_ssid", "your_wifi_password");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }

  mqttClient.setServer("your_mqtt_server", 1883);
  while (!mqttClient.connected()) {
    if (mqttClient.connect("Nano33IoT")) {
      Serial.println("Connected to MQTT broker");
    } else {
      delay(5000);
    }
  }
}

void loop() {
  int ecgValue = analogRead(ECG_PIN);
  float temperature = mlx.readObjectTempC();
  uint32_t irValue = particleSensor.getIR();
  uint32_t redValue = particleSensor.getRed();
  int heartRate = getHeartRate();

  char topic[50];
  sprintf(topic, "sensors/temperature");
  mqttClient.publish(topic, String(temperature).c_str(), true);

  sprintf(topic, "sensors/ecg");
  mqttClient.publish(topic, String(ecgValue).c_str(), true);

  sprintf(topic, "sensors/pulse/ir");
  mqttClient.publish(topic, String(irValue).c_str(), true);

  sprintf(topic, "sensors/pulse/red");
  mqttClient.publish(topic, String(redValue).c_str(), true);

  sprintf(topic, "sensors/heart_rate");
  mqttClient.publish(topic, String(heartRate).c_str(), true);

  delay(1000);
}

