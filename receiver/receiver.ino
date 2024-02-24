#include <math.h>
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoMqttClient.h>
#include <Wire.h>
#include <LoRa.h>
#include <ArduinoJson.h>

#include "hwdefs.h"
#include "credentials.h"

#ifdef USE_DISPLAY
#include <U8g2lib.h>
U8G2_SSD1306_128X64_NONAME_F_HW_I2C *u8g2 = nullptr;
#include "display.h"
#endif

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char topic[]  = "tele/lorasensor/sensor";

struct __attribute__ ((packed)) LoraMsg {
  uint32_t  id;
  uint16_t  seq;
  int16_t temperature;
  int8_t humidity;
  int16_t vbat;
};

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
    // When the power is turned on, a delay is required.
    //delay(1000);

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



    Serial.println("LoRa Receiver " __DATE__ ", " __TIME__);
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(LoRa_frequency)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    }
    LoRa.enableCrc();
}

void loop()
{
  struct LoraMsg loraMsg;
  char buf[64];
  char vbatbuf[16];
  char snrbuf[16];
  uint32_t dispOn = millis();

  for (;;) {
    mqttClient.poll();

  // Wait for packet
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      // received a packet
      Serial.printf("\nReceived packet. Size = %d, expected %d\n", packetSize, sizeof(loraMsg));

      uint8_t *p = (uint8_t *)&loraMsg;
      for (int i = 0; i < packetSize; i++) {
        p[i] = (uint8_t)LoRa.read();
      }
      int rssi = LoRa.packetRssi();
      float snr = LoRa.packetSnr();

      if (loraMsg.id != 0x48764531) {
        Serial.printf("Wrong sensor ID: %x, expected %x", loraMsg.id, 0x48764531);
      }
      else {
        Serial.printf("Counter     = %d\n", loraMsg.seq);
        Serial.printf("Temperature = %d\n", loraMsg.temperature);
        Serial.printf("Humidity    = %d\n", loraMsg.humidity);        // %
        Serial.printf("Vbat        = %d\n", loraMsg.vbat);   // mv

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
        String msg;
        serializeJson(doc, msg);

        mqttClient.beginMessage(topic);
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
