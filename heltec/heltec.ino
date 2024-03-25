
// Turns the 'PRG' button into the power button, long press is off 
//#define HELTEC_POWER_BUTTON   // must be before "#include <heltec.h>"
#define NO_DISPLAY
#include <heltec.h>

// Pause between transmited packets in seconds.
// Set to zero to only transmit a packet when pressing the user button
// Will not exceed 1% duty cycle, even if you set a lower value.
//#define PAUSE               300

// Frequency in MHz. Keep the decimal point to designate float.
// Check your own rules and regulations to see what is legal where you are.
#define FREQUENCY           868.0       // for Europe
// #define FREQUENCY           905.2       // for US

// LoRa bandwidth. Keep the decimal point to designate float.
// Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0 and 500.0 kHz.
#define BANDWIDTH           125.0

// Number from 5 to 12. Higher means slower but higher "processor gain",
// meaning (in nutshell) longer range and more robust against interference. 
#define SPREADING_FACTOR    7

bool rxFlag;
uint8_t rxdata[64];

struct __attribute__ ((packed)) LoraMsg {
  uint32_t  id;
  uint16_t  seq;
  int16_t temperature;
  int8_t humidity;
  int16_t vbat;
};


void setup() {
  heltec_setup();
  Serial.println("Radio init");
  //RADIOLIB_OR_HALT(radio.begin());
  radio.begin();
  // Set the callback function for received packets
  radio.setDio1Action(rx);
  // Set radio parameters
  both.printf("Frequency: %.2f MHz\n", FREQUENCY);
  //RADIOLIB_OR_HALT(radio.setFrequency(FREQUENCY));
  radio.setFrequency(FREQUENCY);
  Serial.printf("Bandwidth: %.1f kHz\n", BANDWIDTH);
  //RADIOLIB_OR_HALT(radio.setBandwidth(BANDWIDTH));
  radio.setBandwidth(BANDWIDTH);
  Serial.printf("Spreading Factor: %i\n", SPREADING_FACTOR);
  //RADIOLIB_OR_HALT(radio.setSpreadingFactor(SPREADING_FACTOR));
  radio.setSpreadingFactor(SPREADING_FACTOR);
  //Serial.printf("TX power: %i dBm\n", TRANSMIT_POWER);
  //RADIOLIB_OR_HALT(radio.setOutputPower(TRANSMIT_POWER));
  // Start receiving
  //RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
  radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);

  rxFlag = false;
}

void loop() {
  heltec_loop();
  
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
