/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * modified by Daniel Bates to include TinyGPS+ 
 */

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <TinyGPS++.h>
// #include <SoftwareSerial.h>

// static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
// The TinyGPS++ object
TinyGPSPlus gps;
// The serial connection to the GPS device
// SoftwareSerial ss(RXPin, TXPin);


/* 
  From http://aprs.gids.nl/nmea/:
   
  $GPGSV
  
  GPS Satellites in view
  
  eg. $GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74
      $GPGSV,3,2,11,14,25,170,00,16,57,208,39,18,67,296,40,19,40,246,00*74
      $GPGSV,3,3,11,22,42,067,42,24,14,311,43,27,05,244,00,,,,*4D

  1    = Total number of messages of this type in this cycle
  2    = Message number
  3    = Total number of SVs in view
  4    = SV PRN number
  5    = Elevation in degrees, 90 maximum
  6    = Azimuth, degrees from true north, 000 to 359
  7    = SNR, 00-99 dB (null when not tracking)
  8-11 = Information about second SV, same as field 4-7
  12-15= Information about third SV, same as field 4-7
  16-19= Information about fourth SV, same as field 4-7
*/

// static const int MAX_SATELLITES = 40;

// TinyGPSCustom totalGPGSVMessages(gps, "GPGSV", 1); // $GPGSV sentence, first element
// TinyGPSCustom messageNumber(gps, "GPGSV", 2);      // $GPGSV sentence, second element
// TinyGPSCustom satsInView(gps, "GPGSV", 3);         // $GPGSV sentence, third element
// TinyGPSCustom satNumber[4]; // to be initialized later
// TinyGPSCustom elevation[4];
// TinyGPSCustom azimuth[4];
// TinyGPSCustom snr[4];

// struct
// {
//   bool active;
//   int elevation;
//   int azimuth;
//   int snr;
// } sats[MAX_SATELLITES];


//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x8C, 0x51, 0x03, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0xFB, 0x20, 0xA6, 0x00, 0xAA, 0x66, 0x7C, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0xB1, 0x55, 0x3A, 0x32, 0x45, 0xE3, 0xD0, 0xDE, 0x2B, 0xB0, 0x4E, 0x07, 0x48, 0x05, 0x40, 0xA3 };
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, LMIC_UNUSED_PIN}  // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
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
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    delay(2000);
    Serial.begin(115200);
    Serial2.begin(GPSBaud);
    Serial.println(F("Starting LoRa"));
    
    

    // #ifdef VCC_ENABLE
    // // For Pinoccio Scout boards
    // pinMode(VCC_ENABLE, OUTPUT);
    // digitalWrite(VCC_ENABLE, HIGH);
    // delay(1000);
    // #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
    
    // Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
    // // Initialize all the uninitialized TinyGPSCustom objects
    // for (int i=0; i<4; ++i)
    // {
    //     satNumber[i].begin(gps, "GPGSV", 4 + 4 * i); // offsets 4, 8, 12, 16
    //     elevation[i].begin(gps, "GPGSV", 5 + 4 * i); // offsets 5, 9, 13, 17
    //     azimuth[i].begin(  gps, "GPGSV", 6 + 4 * i); // offsets 6, 10, 14, 18
    //     snr[i].begin(      gps, "GPGSV", 7 + 4 * i); // offsets 7, 11, 15, 19
    // }

    
    
}

int i_am_alive_Interval = 1000;
long currentMillis;
long previousMillis;

void loop() {
    // LoRa below
    os_runloop_once(); 
    // This sketch displays information every time a new sentence is correctly encoded.
    // LoRa end

    unsigned long currentMillis = millis();
 
    if(currentMillis - previousMillis > i_am_alive_Interval) {
        previousMillis = currentMillis;   

        Serial.println("Ping!");

        // blink LED
        if (digitalRead(2) == true)
        {
            digitalWrite(2, false);
        }
        else 
        {
            digitalWrite(2, true);
        }
    }

    // // GPS below
    // while (Serial2.available() > 0) {
    //     // if (gps.encode(Serial2.read()))
    //     // displayInfo();
    //     Serial.print(char(Serial2.read()));
    // }

    // if (millis() > 5000 && gps.charsProcessed() < 10)
    // {
    //     Serial.println(F("No GPS detected: check wiring."));
    //     // while(true);
    // }
    // GPS end

//   // Dispatch incoming characters
//   if (Serial2.available() > 0)
//   {
//     gps.encode(Serial2.read());
//     if (totalGPGSVMessages.isUpdated())
//     {
//       for (int i=0; i<4; ++i)
//       {
//         int no = atoi(satNumber[i].value());
//         // Serial.print(F("SatNumber is ")); Serial.println(no);
//         if (no >= 1 && no <= MAX_SATELLITES)
//         {
//           sats[no-1].elevation = atoi(elevation[i].value());
//           sats[no-1].azimuth = atoi(azimuth[i].value());
//           sats[no-1].snr = atoi(snr[i].value());
//           sats[no-1].active = true;
//         }
//       }
      
//       int totalMessages = atoi(totalGPGSVMessages.value());
//       int currentMessage = atoi(messageNumber.value());
//       if (totalMessages == currentMessage)
//       {
//         Serial.print(F("Sats=")); Serial.print(gps.satellites.value());
//         Serial.print(F(" Nums="));
//         for (int i=0; i<MAX_SATELLITES; ++i)
//           if (sats[i].active)
//           {
//             Serial.print(i+1);
//             Serial.print(F(" "));
//           }
//         Serial.print(F(" Elevation="));
//         for (int i=0; i<MAX_SATELLITES; ++i)
//           if (sats[i].active)
//           {
//             Serial.print(sats[i].elevation);
//             Serial.print(F(" "));
//           }
//         Serial.print(F(" Azimuth="));
//         for (int i=0; i<MAX_SATELLITES; ++i)
//           if (sats[i].active)
//           {
//             Serial.print(sats[i].azimuth);
//             Serial.print(F(" "));
//           }
        
//         Serial.print(F(" SNR="));
//         for (int i=0; i<MAX_SATELLITES; ++i)
//           if (sats[i].active)
//           {
//             Serial.print(sats[i].snr);
//             Serial.print(F(" "));
//           }
//         Serial.println();

//         for (int i=0; i<MAX_SATELLITES; ++i)
//           sats[i].active = false;
//       }
//     }
//   }


// }

// void displayInfo()
// {
//   Serial.print(F("Location: ")); 
//   if (gps.location.isValid())
//   {
//     Serial.print(gps.location.lat(), 6);
//     Serial.print(F(","));
//     Serial.print(gps.location.lng(), 6);
//   }
//   else
//   {
//     Serial.print(F("INVALID"));
//   }

//   Serial.print(F("  Date/Time: "));
//   if (gps.date.isValid())
//   {
//     Serial.print(gps.date.month());
//     Serial.print(F("/"));
//     Serial.print(gps.date.day());
//     Serial.print(F("/"));
//     Serial.print(gps.date.year());
//   }
//   else
//   {
//     Serial.print(F("INVALID"));
//   }

//   Serial.print(F(" "));
//   if (gps.time.isValid())
//   {
//     if (gps.time.hour() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.hour());
//     Serial.print(F(":"));
//     if (gps.time.minute() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.minute());
//     Serial.print(F(":"));
//     if (gps.time.second() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.second());
//     Serial.print(F("."));
//     if (gps.time.centisecond() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.centisecond());
//   }
//   else
//   {
//     Serial.print(F("INVALID"));
//   }

//   Serial.println();
}


