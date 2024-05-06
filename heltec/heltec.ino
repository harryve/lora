#include <AM2315C.h>
#include <BH1750.h>
#include <heltec_unofficial.h>
#include "loramsg.h"

#define FREQUENCY           868.0
#define BANDWIDTH           125.0
#define SPREADING_FACTOR    7
#define TRANSMIT_POWER      0

#define I2C_SDA                     33
#define I2C_SCL                     34

RTC_DATA_ATTR int counter = 0;
RTC_DATA_ATTR uint32_t runtime = 0;

AM2315C thSensor;
BH1750 lightMeter;


void setup() 
{
  Serial.begin(115200);
  //Serial.println("");
  //Serial.println("initBoard...");

  heltec_ve(true);

  //Serial.printf("Init I2C %ld\n", millis());
  Wire.begin(I2C_SDA, I2C_SCL); //, 100000);
  thSensor.begin();
  lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE);
  //Serial.printf("Sensors done %ld\n", millis());

  //Serial.println("Radio init");
  //RADIOLIB_OR_HALT(radio.begin());
  radio.begin();
  radio.setFrequency(FREQUENCY);
  radio.setBandwidth(BANDWIDTH);
  radio.setSpreadingFactor(SPREADING_FACTOR);
  radio.setOutputPower(TRANSMIT_POWER);
  radio.setCRC(2);
  //Serial.printf("Init done %ld\n", millis());
}

void loop() 
{
  int status;  
  float temp, hum;

  if ((status = thSensor.read()) == AM2315C_OK) {
    temp = thSensor.getTemperature();
    hum = thSensor.getHumidity();
  }
  else {
    temp = 0.0;
    hum = 0.0;
    Serial.printf("AM2315C read error: %d\n", status);
  }

  //  Serial.print("Hum = ");
  //  Serial.print(hum, 1);
  //  Serial.print("%, temperature = ");
  //  Serial.print(temp, 1);
  //  Serial.println(" C");
  uint32_t start = millis();
  while (!lightMeter.measurementReady(true)) {
    yield();
  }
  //Serial.printf("Lightmeter took %d ms\n", millis() - start);

  float lux = lightMeter.readLightLevel();
  //Serial.print("Light: ");
  //Serial.print(lux);
  //Serial.println(" lx");

  //Serial.printf("TX %d\n", millis());
  //heltec_led(50); // 50% brightness is plenty for this LED

  struct LoraMsg loraMsg;
  loraMsg.id = 0x48764532;
  loraMsg.seq = counter;
  loraMsg.temperature = round(temp*10); // Tenths degeree Celcius
  loraMsg.humidity = round(hum);        // %
  loraMsg.illuminance = round(lux);
  loraMsg.vbat = round(heltec_vbat() * 1000.0);   // mv
  loraMsg.runtime = (uint16_t)runtime;

  radio.transmit((uint8_t *)&loraMsg, sizeof(loraMsg));
  Serial.printf("TX done %d\n", millis());

  //heltec_led(0);

  counter += 1;
  runtime = millis();

  heltec_deep_sleep(60);
}

// Can't do Serial or display things here, takes too much time for the interrupt
//void rx() {
//  rxFlag = true;
//}
