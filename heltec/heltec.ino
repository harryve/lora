//#include <Arduino.h>
//#include <Wire.h>
#include <AM2315C.h>
//#define NO_DISPLAY
#include <heltec_unofficial.h>
#define FREQUENCY           868.0
#define BANDWIDTH           125.0
#define SPREADING_FACTOR    7
#define TRANSMIT_POWER      0

#define I2C_SDA                     33
#define I2C_SCL                     34

#ifndef HELTEC_WIRELESS_STICK_LITE
error
#endif

#ifndef HELTEC_NO_DISPLAY
error
#endif

AM2315C amSensor;

bool rxFlag;
uint8_t rxdata[64];

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

  Serial.println("Radio init");
  //RADIOLIB_OR_HALT(radio.begin());
  radio.begin();
  // Set the callback function for received packets
  radio.setDio1Action(rx);
  // Set radio parameters
  Serial.printf("Frequency: %.2f MHz\n", FREQUENCY);
  //RADIOLIB_OR_HALT(radio.setFrequency(FREQUENCY));
  radio.setFrequency(FREQUENCY);
  Serial.printf("Bandwidth: %.1f kHz\n", BANDWIDTH);
  //RADIOLIB_OR_HALT(radio.setBandwidth(BANDWIDTH));
  radio.setBandwidth(BANDWIDTH);
  Serial.printf("Spreading Factor: %i\n", SPREADING_FACTOR);
  //RADIOLIB_OR_HALT(radio.setSpreadingFactor(SPREADING_FACTOR));
  radio.setSpreadingFactor(SPREADING_FACTOR);
  
  Serial.printf("TX power: %i dBm\n", TRANSMIT_POWER);
  radio.setOutputPower(TRANSMIT_POWER);
  // Start receiving
  //RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
  radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);

  rxFlag = false;

  heltec_ve(true);

  Serial.printf("Init AM2315C %ld\n", millis());
  Wire.begin(I2C_SDA, I2C_SCL); //, 100000);
  Serial.printf("Sensor begin %ld\n", millis());
  amSensor.begin();
  Serial.printf("Sensor done %ld\n", millis());
}

void loop() 
{
  static uint32_t lastSensRead = 0;
  int status;  
  float temp, hum, vbat;
  //heltec_loop();

  if (millis() - lastSensRead > 60000) {
    lastSensRead = millis();
 
    //Serial.printf("Read sensor %ld\n", millis());
    if ((status = amSensor.read()) == AM2315C_OK) {
     //Serial.printf("Read done %ld\n", millis());
     temp = amSensor.getTemperature();
     hum = amSensor.getHumidity();
    }
    else {
      temp = 0.0;
      hum = 0.0;
      Serial.print("AM2315C read error: ");
      Serial.println(status);
    }

    Serial.print("Hum = ");
    Serial.print(hum, 1);
    Serial.print("%%, temperature = ");
    Serial.print(temp, 1);
    Serial.println(" C");

    //Serial.printf("Done sensor\n");
  }
  //bool tx_legal = millis() > last_tx + minimum_pause;
  // Transmit a packet every PAUSE seconds or when the button is pressed
  //if ((PAUSE && tx_legal && millis() - last_tx > (PAUSE * 1000)) || button.isS/ingleClick()) {
    // In case of button click, tell user to wait
  //  if (!tx_legal) {
  //    both.printf("Legal limit, wait %i sec.\n", ((minimum_pause - (millis() - last_tx)) / 1000) + 1);
  //    return;
  //  }
  //  both.printf("TX [%s] ", String(counter).c_str());
  //  radio.clearDio1Action();
  //  heltec_led(50); // 50% brightness is plenty for this LED
  //  tx_time = millis();
  //  RADIOLIB(radio.transmit(String(counter++).c_str()));
  //  tx_time = millis() - tx_time;
  //  heltec_led(0);
  //  if (_radiolib_status == RADIOLIB_ERR_NONE) {
  //    both.printf("OK (%i ms)\n", tx_time);
  //  } else {
  //    both.printf("fail (%i)\n", _radiolib_status);
  //  }
  //  // Maximum 1% duty cycle
  //  minimum_pause = tx_time * 100;
  //  last_tx = millis();
  //  radio.setDio1Action(rx);
  //  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
  //}

  // If a packet was received, display it and the RSSI and SNR

  if (rxFlag) {
    heltec_led(100);
    struct LoraMsg loraMsg;
    rxFlag = false;
    int l = radio.getPacketLength();
    if (l == sizeof(loraMsg)) {
      radio.readData(rxdata, l);
      //if (_radiolib_status == RADIOLIB_ERR_NONE) {
      //both.printf("RX(%d) [%s]\n", l, rxdata.c_str());
      both.printf("RX len = %d\n", l);
      both.printf("  RSSI: %.2f dBm\n", radio.getRSSI());
      both.printf("  SNR: %.2f dB\n", radio.getSNR());
      if (l == sizeof(loraMsg)) {
        radio.readData((uint8_t *)&loraMsg, l);
        if (loraMsg.id != 0x48764531) {
          Serial.printf("Wrong sensor ID: %x, expected %x", loraMsg.id, 0x48764531);
        }
        else {
          Serial.printf("Counter     = %d\n", loraMsg.seq);
          Serial.printf("Temperature = %d\n", loraMsg.temperature);
          Serial.printf("Humidity    = %d\n", loraMsg.humidity);        // %
          Serial.printf("Vbat        = %d\n", loraMsg.vbat);   // mv
        }
      }
      else {
        Serial.printf("Wrong size\n");
        radio.readData(rxdata, l); // Flush
      }
    }
    radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
    heltec_led(0);
  }
}

// Can't do Serial or display things here, takes too much time for the interrupt
void rx() {
  rxFlag = true;
}
