//final one
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#define SERVER_IP "192.168.217.136:8000"

#ifndef STASSID
#define STASSID "Disha"
#define STAPSK "dishaa.2233"
#endif

#if defined(_AVR_ATmega328P) || defined(AVR_ATmega168_)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100];   //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100];   //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength;   //data length
int32_t spo2;           //SPO2 value
int8_t validSPO2;       //indicator to show if the SPO2 calculation is valid
int32_t heartRate;      //heart rate value
int8_t validHeartRate;  //indicator to show if the heart rate calculation is valid

byte pulseLED = 5;  //Must be on PWM pin
byte readLED = 4;   //Blinks with each data read

DynamicJsonDocument doc(1024);
String postData;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  // initialize serial communication at 115200 bits per second:
  WiFi.begin(STASSID, STAPSK);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);
  Serial.println("Adafruit MLX90614 test");

  if (!mlx.begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1)
      ;
  };

  Serial.print("Emissivity = ");
  Serial.println(mlx.readEmissivity());
  Serial.println("================================================");

  // Initialize sensor
  if (!particleSensor.begin(Wire))  //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1)
      ;
  }

  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  while (Serial.available() == 0)
    ;  //wait until user presses a key
  Serial.read();

  byte ledBrightness = 60;  //Options: 0=Off to 255=50mA
  byte sampleAverage = 4;   //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;         //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100;    //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;     //Options: 69, 118, 215, 411
  int adcRange = 4096;      //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);  //Configure sensor with these settings
}

void loop() {
  // put your main code here, to run repeatedly:
  bufferLength = 100;  //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  Serial.print("Waiting for raw Data");
  for (byte i = 0; i < bufferLength; i++) {
    while (particleSensor.available() == false)  //do we have new data?
      particleSensor.check();                    //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();  //We're finished with this sample so move to next sample

    // Serial.print(F("red="));
    // Serial.print(redBuffer[i], DEC);
    // Serial.print(F(", ir="));
    // Serial.println(irBuffer[i], DEC);
    Serial.print(".");
  }
  Serial.println();

  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  while (1) {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++) {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++) {
      while (particleSensor.available() == false)  //do we have new data?
        particleSensor.check();                    //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED));  //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample();  //We're finished with this sample so move to next sample

      if (validSPO2) {
        Serial.print("SPO2: ");
        Serial.print(spo2);

        Serial.print(" HR=");
        Serial.print(heartRate, DEC);

        Serial.print(" Body Temp= ");
        Serial.print(mlx.readObjectTempC());
        Serial.println("*C");
        if ((WiFi.status() == WL_CONNECTED)) {
          WiFiClient client;
          HTTPClient http;

          Serial.print("[HTTP] begin...\n");
          // configure traged server and url
          http.begin(client, "http://" SERVER_IP "/api/patient/data");  // HTTP
          
          http.addHeader("Content-Type", "application/json");

          Serial.print("[HTTP] POST...\n");
          // start connection and send HTTP header and body
          doc["username"] = "Disha1";
          doc["heart_rate"] = heartRate;
          doc["temperature"] = (int)mlx.readObjectTempC();
          doc["spo2"] = spo2;
          postData = "";
          serializeJson(doc, postData);
          Serial.println(postData);

          int httpCode = http.POST(postData);

          // httpCode will be negative on error
          if (httpCode > 0) {
            // HTTP header has been send and Server response header has been handled
            Serial.printf("[HTTP] POST... code: %d\n", httpCode);

            // file found at server
            if (httpCode == HTTP_CODE_OK) {
              const String& payload = http.getString();
              Serial.println("received payload:\n<<");
              Serial.println(payload);
              Serial.println(">>");
            }
          } else {
            Serial.println(httpCode);
            Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
          }
          http.end();
        }
      }
      // delay(2000);
    }
    Serial.println("Gathering new sample");

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
}
