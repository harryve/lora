#define SENSOR1_ID 0x48764531
#define SENSOR2_ID 0x48764532

struct __attribute__ ((packed)) LoraMsg {
  uint32_t id;
  uint16_t seq;
  int16_t  temperature;
  int8_t   humidity;
  int16_t  vbat;
  uint16_t runtime;
  uint16_t illuminance;
};

