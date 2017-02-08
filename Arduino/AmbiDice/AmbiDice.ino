#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <Wire.h>
#include "OSCMessage.h"
#include <array>
#include "helper_3dmath.h" // 3D vector class definitions
#include "MPU6050.h"       // IMU library

extern "C" {
#include <user_interface.h> // Contains method "wifi_softap_get_station_num()"
}


////////////////////////////////////////////////////////////////////////////////
// Configurable constants //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
const char ssid[]     = "AmbiDice";
const char password[] = "seriousResearch";

const uint16_t LOCAL_PORT  = 8888;
const uint16_t REMOTE_PORT = 9876;
const char     REMOTE_IP[] = "192.168.4.2"; // DO NOT CHANGE IF NOT SURE!

const uint16_t INTERVAL = 100; // Interval for periodic messages in milliseconds


////////////////////////////////////////////////////////////////////////////////
// Global variables and constants //////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
WiFiUDP udp;
MPU6050 mpu;

// Definition of accelerometer values for each face (=calibration)
std::array<VectorInt16*, 12> faces =  {
  new VectorInt16(  -330,   -310,  16180),
  new VectorInt16( 13780,   4330,   7300),
  new VectorInt16(  -150,  14570,   7130),
  new VectorInt16(-14070,   4520,   6910),
  new VectorInt16( -8850, -12000,   6930),
  new VectorInt16(  8390, -12100,   7150),
  new VectorInt16(  -100,    -20, -16870),
  new VectorInt16(  8480,  11890,  -7550),
  new VectorInt16( 14030,  -4500,   7610),
  new VectorInt16(   160, -14870, - 7780),
  new VectorInt16(-13800,  -4870,  -8080),
  new VectorInt16( -8710,  11680,  -7850)
};

uint32_t pMillis = 0;                  // Needed for periodic sending
int16_t  x = 0, y = 0, z = 0;          // Latest accelerometer values
int64_t  sumX = 0, sumY = 0, sumZ = 0; // Summed accelerometer values
uint32_t nMeasurements = 0;            // Number of measurements to average over
int8_t   currentFace;                  // Current face of the dice

boolean  connected = false;            // Connection state
uint32_t lastConnectionMillis = 0;     // Timer value of last connection

const uint16_t CON_DELAY = 1000;       // Delay before sending data after connection


////////////////////////////////////////////////////////////////////////////////
// Setup and initialization ////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void setup() {
  delay(1000); // Allow for ESP8266 to finish init

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Wire.begin(2, 0); // Init i2c with Data(2) and Clock(0) pins

  // Initalize MPU6050
  mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16); // Enable up to +/- 16g
  mpu.setSleepEnabled(false);

  udp.begin(LOCAL_PORT); // Initalize UDP

  pinMode(LED_BUILTIN, OUTPUT);
}


////////////////////////////////////////////////////////////////////////////////
// Loop ////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void loop() {
  if (wifi_softap_get_station_num() > 0 ) {
    if (!connected) {
      connected = true;
      digitalWrite(LED_BUILTIN, LOW);
      lastConnectionMillis = millis();
    } else {
      if (millis() - lastConnectionMillis > CON_DELAY) {
        mpu.getAcceleration(&x, &y, &z); // Get current sensor values

        sendRaw(REMOTE_IP);

        // Sum up the values and increase measurement counter
        sumX += x;
        sumY += y;
        sumZ += z;
        nMeasurements++;

        // Send current face periodically
        uint32_t cMillis = millis();
        if ((cMillis - pMillis >= INTERVAL)) {
          pMillis = cMillis;
          int16_t averageX = sumX / nMeasurements;
          int16_t averageY = sumY / nMeasurements;
          int16_t averageZ = sumZ / nMeasurements;

          // Calculate current face using cosine similarity
          VectorInt16 v(averageX, averageY, averageZ);
          float maxSimilarity = -1.0f;
          uint8_t newFace = 0;
          for (uint8_t i = 0; i < faces.size(); i++) {
            float similarity = v.getCosineSimilarityTo(faces[i]);
            if (similarity > maxSimilarity) {
              newFace = i;
              maxSimilarity = similarity;
            }
          }
          currentFace = newFace;
          sendFace(REMOTE_IP);

          // Reset values
          sumX = 0;
          sumY = 0;
          sumZ = 0;
          nMeasurements = 0;
        }
      }
    }
  } else {
    connected = false;
    digitalWrite(LED_BUILTIN, HIGH);
  }
}


////////////////////////////////////////////////////////////////////////////////
// OSC sending methods /////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void sendFace(const char* ip) {
  OSCMessage msg("/ambidice/face");
  msg.add(currentFace);

  udp.beginPacket(ip, REMOTE_PORT);
  msg.send(udp);
  udp.endPacket();
}

void sendRaw(const char* ip) {
  OSCMessage msg("/ambidice/raw");
  msg.add(x);
  msg.add(y);
  msg.add(z);

  udp.beginPacket(ip, REMOTE_PORT);
  msg.send(udp);
  udp.endPacket();
}
