#include <Arduino.h>

typedef struct sensorData {
  uint16_t temperature;
  uint16_t humidity;
  uint16_t pressure;
  uint16_t battery;
  uint16_t supply;
};

#define PACKET_SIZE sizeof(sensorData)

typedef union TTN_Payload {
  sensorData levels;
  byte LoRaPacketBytes[PACKET_SIZE];
};