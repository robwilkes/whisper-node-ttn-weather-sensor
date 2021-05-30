
# whisper-node-ttn-weather-sensor
This should work on any Whisper Node LoRa as is, and on any ATMega328p board with minor modifications.

##  Set region/band
Set the correct region/band for your use in

.pio/libdeps/whispernode/MCCI LoRaWAN LMIC library/project_config/lmic_project_config.h:
```
#define CFG_au915 1
```
## Configuring The Things Network specifics/keys
Copy include/sensor_specifics.sample.h to include/sensor_specifics.h.
Replace APPEUI, DEVEUI, and APPKEY with correct values from TTN console.

include/sensor_specifics.h:
```
static  const  u1_t  PROGMEM  APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // reverse order (little endian)
static  const  u1_t  PROGMEM  DEVEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // reverse order (little endian)
static  const  u1_t  PROGMEM  APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
```
## Size Optimisation
If you need to support more sensors, and simply wish to know how to optimise it, the below changes are safe to make.
(The maximum message size could be reduced even further if desired)

.pio/libdeps/whispernode/MCCI LoRaWAN LMIC library/src/lmic/config.h:
```
#define DISABLE_PING                // LoRaWAN Class B support not needed/implemented
#define DISABLE_BEACONS             // LoRaWAN Class B support not needed/implemented
#define LMIC_MAX_FRAME_LENGTH 64    // Sets maximum LoRaWAN message size to 64 bytes
```

Before:
RAM:   [========  ]  80.6% (used 1651 bytes from 2048 bytes)
Flash: [========= ]  91.7% (used 29576 bytes from 32256 bytes)

After:
RAM:   [======    ]  59.5% (used 1218 bytes from 2048 bytes)
Flash: [========  ]  83.1% (used 26812 bytes from 32256 bytes)

