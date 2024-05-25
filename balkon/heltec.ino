#include <ArduinoOTA.h>
#include <WiFi.h>
#include <AM2315C.h>
#include <BH1750.h>
#include <heltec_unofficial.h>
#include "loramsg.h"
#include "credentials.h"

#define FREQUENCY           868.0
#define BANDWIDTH           125.0
#define SPREADING_FACTOR    7
#define TRANSMIT_POWER      0

#define I2C_SDA                     33
#define I2C_SCL                     34
#define CHARGE_ADC          GPIO_NUM_3

RTC_DATA_ATTR int counter = 0;
RTC_DATA_ATTR uint32_t runtime = 0;

AM2315C thSensor;
BH1750 lightMeter;

static int lightMeterStart;

static bool isCharging()
{
  if (analogRead(CHARGE_ADC) > 500.0) {
    return true;
  }
  return false;
}

void setup()
{
  Serial.begin(115200);
  Serial.printf("\nInit balkon sensor %s %s at %ld\n", __DATE__, __TIME__, millis());

  heltec_ve(true);

  Serial.printf("Init I2C %ld\n", millis());
  Wire.begin(I2C_SDA, I2C_SCL); //, 100000);
  thSensor.begin();
  thSensor.requestData();

  lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE);
  lightMeterStart = millis();
  Serial.printf("Sensors done %ld\n", millis());

  if ((radio.begin() != RADIOLIB_ERR_NONE) ||
    (radio.setFrequency(FREQUENCY) != RADIOLIB_ERR_NONE) ||
    (radio.setBandwidth(BANDWIDTH) != RADIOLIB_ERR_NONE) ||
    (radio.setSpreadingFactor(SPREADING_FACTOR) != RADIOLIB_ERR_NONE) ||
    (radio.setOutputPower(TRANSMIT_POWER) != RADIOLIB_ERR_NONE) ||
    (radio.setCRC(2) != RADIOLIB_ERR_NONE)) {
    Serial.printf("Radio init failed\n");
    heltec_led(50);
    delay(500);
    heltec_deep_sleep(60);
  }

  Serial.printf("Radio init done %ld\n", millis());

  if (isCharging()) {
    Serial.print("Connect to WiFi");
    WiFi.setHostname("balkonsensor");
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(500);
    }
    Serial.print("\nConnected\n");
    heltec_led(50);
    ArduinoOTA.setHostname("balkonsensor");
    ArduinoOTA.onStart([]() {
      Serial.println("Start");
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });    
    ArduinoOTA.begin();
  }
}

void loop() 
{
  if (isCharging()) {
    ChargeLoop();
  }
  else {
    AccuLoop();
  }
}

void ChargeLoop()
{
  ArduinoOTA.handle();
}

void AccuLoop()
{
  int status;  
  float temp, hum;

  Serial.printf("loop %ld\n", millis());
  thSensor.readData();
  if ((status = thSensor.convert()) == AM2315C_OK) {
    temp = thSensor.getTemperature();
    hum = thSensor.getHumidity();
  }
  else {
    temp = 0.0;
    hum = 0.0;
    Serial.printf("AM2315C read error: %d\n", status);
  }
  Serial.printf("ht sensor done %ld\n", millis());

  while (millis() - lightMeterStart < 120) {
    delay(1);
  }
  Serial.printf("Lightmeter done %ld\n", millis());
  float lux = lightMeter.readLightLevel();

  struct LoraMsg loraMsg;
  loraMsg.id = SENSOR2_ID;
  loraMsg.seq = counter;
  loraMsg.temperature = round(temp*10); // Tenths degeree Celcius
  loraMsg.humidity = round(hum);        // %
  loraMsg.illuminance = round(lux);
  loraMsg.vbat = round(heltec_vbat() * 1000.0);   // mv
  loraMsg.runtime = (uint16_t)runtime;

  radio.transmit((uint8_t *)&loraMsg, sizeof(loraMsg));
  Serial.printf("TX done (t=%d, h=%d, l=%d) %d\n", loraMsg.temperature, loraMsg.humidity, loraMsg.illuminance, millis());

  counter += 1;
  runtime = millis();

  heltec_deep_sleep(5 * 60);
}
