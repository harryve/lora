//Board: TTGO LoRa32-OLED

#include <math.h>
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoMqttClient.h>
#include <Wire.h>
#include <LoRa.h>
#include <ArduinoJson.h>

#include "hwdefs.h"
#include "credentials.h"
#include "loramsg.h"

#ifdef USE_DISPLAY
#include <U8g2lib.h>
U8G2_SSD1306_128X64_NONAME_F_HW_I2C *u8g2 = nullptr;
#include "display.h"
#endif

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char topic1[]  = "tele/lorasensor/sensor";
const char topic2[]  = "tele/lorasensor2/sensor";

void setup()
{
  Serial.begin(115200);
  Serial.println("initBoard....");
  SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
  Wire.begin(I2C_SDA, I2C_SCL); //, 100000);

#ifdef BOARD_LED
  pinMode(BOARD_LED, OUTPUT);
  digitalWrite(BOARD_LED, LED_ON);
#endif

  DisplayInit();

  Serial.print("Connect to WiFi");
  WiFi.setHostname("LoRaBridge");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(5000);
  }
  Serial.print("\nConnected\n");

  mqttClient.setId("LoRaBridge");
  Serial.print("Attempting to connect to the MQTT broker: ");
  if (!mqttClient.connect("mqtt.harry.thuis", 1883)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
  }
  else { 
    Serial.println("Connected to the MQTT broker!");
    Serial.println();
  }

  Serial.println("LoRa Bridge " __DATE__ ", " __TIME__);
  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
  if (!LoRa.begin(LoRa_frequency)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.enableCrc();
}

int check_sensor(uint32_t id)
{
  switch(id) {
    case SENSOR1_ID:
      return 1;
      
    case SENSOR2_ID:
      return 2;
    
    default:
      Serial.printf("Wrong sensor ID: %x\n", id);
  }
  return 0;
}
void loop()
{
  struct LoraMsg loraMsg;
  char buf[64];
  char vbatbuf[16];
  char snrbuf[16];
  uint32_t dispOn = millis();
  int sensor;

  unsigned long previousMillis = millis();
  for (;;) {
    
    unsigned long currentMillis = millis();
    // if WiFi is down, try reconnecting
    if (((WiFi.status() != WL_CONNECTED) || (mqttClient.connected() == 0)) && (currentMillis - previousMillis >= 60000)) {
      Serial.print(millis());
      Serial.println("Reconnecting to WiFi...");
      mqttClient.stop();
      WiFi.disconnect();
      WiFi.reconnect();
      previousMillis = currentMillis;
      if (!mqttClient.connect("mqtt.harry.thuis", 1883)) {
        Serial.print("MQTT connection failed! Error code = ");
        Serial.println(mqttClient.connectError());
      }
      else { 
        Serial.println("Connected to the MQTT broker!");
        Serial.println();
      }
    }
    mqttClient.poll();

  // Wait for packet
  int packetSize = LoRa.parsePacket();
  if (packetSize == sizeof(loraMsg)) {
    // received a packet
    uint8_t *p = (uint8_t *)&loraMsg;
    for (int i = 0; i < packetSize; i++) {
      p[i] = (uint8_t)LoRa.read();
    }
    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();

    if ((sensor = check_sensor(loraMsg.id)) > 0) {
      Serial.printf("Sensor      = %d\n", sensor);
      Serial.printf("Counter     = %d\n", loraMsg.seq);
      Serial.printf("Temperature = %d\n", loraMsg.temperature);
      Serial.printf("Humidity    = %d\n", loraMsg.humidity);        // %
      Serial.printf("Vbat        = %d\n", loraMsg.vbat);   // mv
      Serial.printf("runtime     = %d\n", loraMsg.runtime);   // ms
      Serial.printf("illuminance = %d\n", loraMsg.illuminance);
#ifdef USE_DISPLAY
        if (u8g2) {
          u8g2->sleepOff();
          u8g2->clearBuffer();
          snprintf(buf, sizeof(buf), "Rx %d %d", loraMsg.seq, rssi);
          u8g2->drawStr(0, 20, buf);
          snprintf(buf, sizeof(buf), "T=%d,  RV=%d", loraMsg.temperature, loraMsg.humidity);
          u8g2->drawStr(0, 40, buf);
          snprintf(buf, sizeof(buf), "Vbat=%d", loraMsg.vbat);
          u8g2->drawStr(0, 60, buf);
          u8g2->sendBuffer();
          dispOn = millis();
        }
#endif
#ifdef BOARD_LED
        digitalWrite(BOARD_LED, LED_ON);
#endif
        JsonDocument doc;
        doc["counter"] = loraMsg.seq;
        int frag = loraMsg.temperature % 10;
        snprintf(buf, sizeof(buf), "%d.%d", loraMsg.temperature / 10, abs(frag));
        doc["temp"] = buf;
        doc["hum"] = loraMsg.humidity;        // %
        snprintf(vbatbuf, sizeof(vbatbuf), "%d.%03d", loraMsg.vbat / 1000, loraMsg.vbat % 1000);
        doc["vbat"] = vbatbuf;
        doc["rssi"] = rssi;
        snprintf(snrbuf, sizeof(snrbuf), "%.1f", snr);
        doc["snr"] = snrbuf;
        doc["runtime"] = loraMsg.runtime;
        doc["illuminance"] = loraMsg.illuminance;
        String msg;
        serializeJson(doc, msg);

        mqttClient.beginMessage(sensor == 1 ? topic1 : topic2);
        mqttClient.print(msg.c_str());
        mqttClient.endMessage();
      }
    }
    else {
      if (millis() - dispOn > 30000) {
        u8g2->sleepOn();
#ifdef BOARD_LED
        digitalWrite(BOARD_LED, LED_OFF);
#endif
      } 
    }
  }
}
