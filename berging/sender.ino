#include <math.h>
#include <Arduino.h>
#include <Wire.h>
#include <LoRa.h>
#include <ESP32AnalogRead.h>
#include <AM2315C.h>
#include "loramsg.h"
#include "hwdefs.h"

RTC_DATA_ATTR int counter = 0;
RTC_DATA_ATTR uint32_t runtime = 0;

AM2315C sensor;
ESP32AnalogRead adc;

#ifdef USE_DISPLAY
#include <U8g2lib.h>
U8G2_SSD1306_128X64_NONAME_F_HW_I2C *u8g2 = nullptr;
#include "display.h"
#endif

void GoToSleep()
{
    Serial.println("Going to sleep");

    if (u8g2) {
      u8g2->sleepOn();
    }
    LoRa.sleep();

    pinMode(RADIO_CS_PIN, INPUT);
    pinMode(RADIO_RST_PIN, INPUT);
    pinMode(RADIO_DIO0_PIN, INPUT);
    pinMode(RADIO_CS_PIN, INPUT);
    pinMode(I2C_SDA, INPUT);
    pinMode(I2C_SDA, INPUT);
    pinMode(I2C_SCL, INPUT);
    pinMode(OLED_RST, INPUT);
    pinMode(RADIO_SCLK_PIN, INPUT);
    pinMode(RADIO_MISO_PIN, INPUT);
    pinMode(RADIO_MOSI_PIN, INPUT);
    pinMode(SDCARD_MOSI, INPUT);
    pinMode(SDCARD_MISO, INPUT);
    pinMode(SDCARD_SCLK, INPUT);
    pinMode(SDCARD_CS, INPUT);
#ifdef BOARD_LED
    pinMode(BOARD_LED, INPUT);
#endif
    pinMode(ADC_PIN, INPUT);

    runtime = millis();
    esp_sleep_enable_timer_wakeup(5*60 * (1000*1000));
    esp_deep_sleep_start();
}

void setup()
{
    Serial.begin(115200);
    Serial.println("initBoard....");
    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
    Wire.begin(I2C_SDA, I2C_SCL, 100000);

#ifdef BOARD_LED
    pinMode(BOARD_LED, OUTPUT);
    digitalWrite(BOARD_LED, LED_ON);
#endif

    sensor.begin();
    adc.attach(ADC_PIN);

    if (counter == 0) {
      DisplayInit();
    }

    // When the power is turned on, a delay is required.
    //delay(1000);

    Serial.println("LoRa Sender " __DATE__ ", " __TIME__);
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(LoRa_frequency)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    }
}

void loop()
{
  int status;
  float temp, hum, vbat;

  vbat = 2 * adc.readVoltage();
  Serial.println("Voltage = " + String(vbat));

  if ((status = sensor.read()) == AM2315C_OK) {
    temp = sensor.getTemperature();
    hum = sensor.getHumidity();
  }
  else {
    temp = 0.0;
    hum = 0.0;
    Serial.print("AM2315C read error: ");
    Serial.println(status);
  }

  Serial.print("Hum = ");
  Serial.print(hum, 1);
  Serial.print("%, temperature = ");
  Serial.print(temp, 1);
  Serial.println(" C");

  Serial.print("Sending packet: ");
  Serial.println(counter);

  // send packet
  struct LoraMsg loraMsg;
  loraMsg.id = 0x48764531;              // HvE1
  loraMsg.seq = counter;
  loraMsg.temperature = round(temp*10); // Tenths degeree Celcius
  loraMsg.humidity = round(hum);        // %
  loraMsg.vbat = round(vbat* 1000.0);   // mv
  loraMsg.runtime = (uint16_t)runtime;
  loraMsg.illuminance = 0;
  
  LoRa.enableCrc();
  LoRa.beginPacket();
  LoRa.write((const uint8_t *)&loraMsg, sizeof(loraMsg));
  //LoRa.print("T=");
  //LoRa.print(temp);
  //LoRa.print(",  c=");
  //LoRa.print(counter);
  LoRa.endPacket();

#ifdef USE_DISPLAY
  if (u8g2) {
      char buf[256];
      u8g2->clearBuffer();
      u8g2->drawStr(0, 12, "Transmitting: OK!");
      snprintf(buf, sizeof(buf), "T=%.1f,  RV=%.0f", temp, hum);
      u8g2->drawStr(0, 30, buf);
      u8g2->sendBuffer();
      delay(5000);
  }
#endif
    counter++;

    GoToSleep();
}
