#include <Arduino.h>           
#include <T2WhisperNode.h>     // Talk2 Library for Board
#include <LowPower.h>          // Low Power Library for ATmega
#include <lmic.h>              // MCCI LMIC
#include <hal/hal.h>           // MCCI LMIC
#include <SPI.h>               // MCCI LMIC
#include <Wire.h>              // I2C for SHT31 & BMP180
#include "ttn_payload.h"       // Custom Payload Struct
#include "sensor_specifics.h"  // Sensor Specific Secrets and Defines

#define SERIAL_BAUDRATE 115200 // Define serial baudrate
#define DEBUG false            // Additional Serial port debug output
#define DEBUG_LORA false       // LoRaWAN Serial port debug output
#define DEBUG_SERIAL if(DEBUG)Serial
#define DEBUG_LORA_SERIAL if(DEBUG_LORA)Serial

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 300;

// Which TTN application port to send on
const int TTN_PORT = 1;

// Pin mapping for Talk2 Whisper Node
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 7,
    .dio = {2, A2, LMIC_UNUSED_PIN},
};

// Include Sensor Libraries and Initialise Objects
#ifdef SHT31
  #include <Adafruit_SHT31.h>    // SHT31 Library
  Adafruit_SHT31 sht31 = Adafruit_SHT31();
#endif
#ifdef BMP180
  #include <Adafruit_BMP085.h>   // BMP085/BMP180 Library
  Adafruit_BMP085 bmp; // BMP180 I2C
#endif

// Define application and device keys
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

T2Flash myFlash;
TTN_Payload data;

static osjob_t sendjob;

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        DEBUG_LORA_SERIAL.print('0');
    DEBUG_LORA_SERIAL.print(v, HEX);
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        DEBUG_LORA_SERIAL.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        #ifdef SHT31
          data.levels.temperature = LMIC_f2sflt16(float(sht31.readTemperature())/100);
          data.levels.humidity = LMIC_f2sflt16(float(sht31.readHumidity())/100);
        #endif
        #ifdef BMP180
          data.levels.pressure = bmp.readPressure();
        #endif
        data.levels.battery = T2Utils::readVoltage(T2_WPN_VBAT_VOLTAGE, T2_WPN_VBAT_CONTROL);
        data.levels.supply = T2Utils::readVoltage(T2_WPN_VIN_VOLTAGE, T2_WPN_VIN_CONTROL);
        LMIC_setTxData2(TTN_PORT, data.LoRaPacketBytes, sizeof(data.LoRaPacketBytes), 0);
        DEBUG_LORA_SERIAL.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    DEBUG_LORA_SERIAL.print(os_getTime());
    DEBUG_LORA_SERIAL.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            DEBUG_LORA_SERIAL.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            DEBUG_LORA_SERIAL.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            DEBUG_LORA_SERIAL.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            DEBUG_LORA_SERIAL.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            DEBUG_LORA_SERIAL.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            DEBUG_LORA_SERIAL.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              DEBUG_LORA_SERIAL.print("netid: ");
              DEBUG_LORA_SERIAL.println(netid, DEC);
              DEBUG_LORA_SERIAL.print("devaddr: ");
              DEBUG_LORA_SERIAL.println(devaddr, HEX);
              DEBUG_LORA_SERIAL.print("AppSKey: ");
              #ifdef DEBUG_LORA
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  DEBUG_LORA_SERIAL.print("-");
                printHex2(artKey[i]);
              }
              DEBUG_LORA_SERIAL.println("");
              DEBUG_LORA_SERIAL.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              DEBUG_LORA_SERIAL.print("-");
                      printHex2(nwkKey[i]);
              }
              DEBUG_LORA_SERIAL.println();
              #endif
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
            // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     DEBUG_SERIAL.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            DEBUG_LORA_SERIAL.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            DEBUG_LORA_SERIAL.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            DEBUG_LORA_SERIAL.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              DEBUG_LORA_SERIAL.println(F("Received ack"));
            if (LMIC.dataLen) {
              DEBUG_LORA_SERIAL.print(F("Received "));
              DEBUG_LORA_SERIAL.print(LMIC.dataLen);
              DEBUG_LORA_SERIAL.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            for (int i=0; i<int(TX_INTERVAL/8); i++) {
              // Use library from https://github.com/rocketscream/Low-Power
              DEBUG_SERIAL.println(F("8s Deep Sleep Loop"));
              DEBUG_SERIAL.flush();
              LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
            }
            do_send(&sendjob);
            break;
        case EV_LOST_TSYNC:
            DEBUG_LORA_SERIAL.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            DEBUG_LORA_SERIAL.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            DEBUG_LORA_SERIAL.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            DEBUG_LORA_SERIAL.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            DEBUG_LORA_SERIAL.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    DEBUG_SERIAL.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            DEBUG_LORA_SERIAL.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            DEBUG_LORA_SERIAL.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            DEBUG_LORA_SERIAL.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            DEBUG_LORA_SERIAL.print(F("Unknown event: "));
            DEBUG_LORA_SERIAL.println((unsigned) ev);
            break;
    }
}

void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    while (DEBUG && !Serial) {} // If DEBUG enabled wait for Serial
    DEBUG_SERIAL.println(F("Starting"));

    // Flash - We’re not using, so just power it down to save energy
    myFlash.init(T2_WPN_FLASH_SPI_CS);
    myFlash.powerDown();

    #ifdef SHT31
      if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
        DEBUG_SERIAL.println("Couldn't find SHT31");
      }
    #endif
    
    #ifdef BMP180
      if (!bmp.begin()) {
        DEBUG_SERIAL.println("Not connected with BMP180/BMP085 sensor, check connections ");
      }
    #endif
    
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setLinkCheckMode(1); // maybe disable?
    LMIC_setDrTxpow(DR_SF7,14);
    LMIC_selectSubBand(1);
    LMIC_setAdrMode(1); // maybe disable?

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
