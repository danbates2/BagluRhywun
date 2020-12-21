/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * modified by Daniel Bates to include TinyGPS+, and more! 
 */

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <TinyGPS++.h>

#define uS_TO_S_FACTOR 1000000  //Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  55        //Time ESP32 will go to sleep (in seconds)
RTC_DATA_ATTR int bootCount = 0;

static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;

int i_am_alive_Interval = 1000;
long currentMillis;
long previousMillis;

// #ifdef COMPILE_REGRESSION_TEST
// # define FILLMEIN 0
// #else
// # warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
// # define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
// #endif

// This EUI must be in little-endian format.
static const u1_t PROGMEM APPEUI[8]={ 0x8C, 0x51, 0x03, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format.
static const u1_t PROGMEM DEVEUI[8]={ 0xFB, 0x20, 0xA6, 0x00, 0xAA, 0x66, 0x7C, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format.
static const u1_t PROGMEM APPKEY[16] = { 0xB1, 0x55, 0x3A, 0x32, 0x45, 0xE3, 0xD0, 0xDE, 0x2B, 0xB0, 0x4E, 0x07, 0x48, 0x05, 0x40, 0xA3 };
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

static uint8_t mydata[] = "Wedi Baglu Rhywun!";
static osjob_t sendjob;

// duty cycle for testing.
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, LMIC_UNUSED_PIN}
};

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void do_send(osjob_t* j) {
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
    Serial.begin(115200);
    delay(2000);
    pinMode(15, INPUT);
    // while(1) {
    //     Serial.println(digitalRead(15));
    //     delay(100);
    // }
    // ++bootCount;
    // Serial.println("Boot number: " + String(bootCount));

    //Print the wakeup reason for ESP32    
    print_wakeup_reason();

    // Serial.println(F("Starting LoRa"));
    // os_init();
    // LMIC_reset();
    // // Start job (sending automatically starts OTAA too)
    // do_send(&sendjob);
    
    
    // Serial2.begin(GPSBaud);
    // Serial.println(F("Starting GPS"));

    // Set sleep timer to TIME_TO_SLEEP seconds
	esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
	Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
	" Seconds");

    // // wake up on GPIO15 Interrupt going low.
    // esp_sleep_enable_ext0_wakeup(GPIO_NUM_15,0);
    // esp_sleep_enable_ext0_wakeup(GPIO_NUM_26,0);


	// // //Go to sleep now
	esp_deep_sleep_start();


}

void loop() {
    
    // LoRa below
    // This sketch displays information every time a new sentence is correctly encoded.
    // os_runloop_once();
    // LoRa end

    unsigned long currentMillis = millis();
 
    if(currentMillis - previousMillis > i_am_alive_Interval) {
        previousMillis = currentMillis;   

        // Serial.println("Ping!");

        // // blink LED
        // if (digitalRead(2) == true)
        // {
        //     digitalWrite(2, false);
        // }
        // else 
        // {
        //     digitalWrite(2, true);
        // }
    }

    // GPS below
    while (Serial2.available() > 0) {
        if (gps.encode(Serial2.read()))
        displayInfo();
        // Serial.print(char(Serial2.read())); // debug
    }

    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
        Serial.println(F("No GPS detected: check wiring."));
        // while(true);
    }
    // GPS end
    
}


void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

void print_wakeup_reason(){
	esp_sleep_source_t wakeup_reason;
	wakeup_reason = esp_sleep_get_wakeup_cause();
    Serial.println(wakeup_reason);
	switch(wakeup_reason)
	{
		case ESP_SLEEP_WAKEUP_UNDEFINED  : Serial.println("ESP_SLEEP_WAKEUP_UNDEFINED"); break;
        case ESP_SLEEP_WAKEUP_ALL  : Serial.println("ESP_SLEEP_WAKEUP_ALL"); break;
        case ESP_SLEEP_WAKEUP_EXT0  : Serial.println("ESP_SLEEP_WAKEUP_EXT0"); break;
        case ESP_SLEEP_WAKEUP_EXT1  : Serial.println("ESP_SLEEP_WAKEUP_EXT1"); break;
		case ESP_SLEEP_WAKEUP_TIMER  : Serial.println("Wakeup caused by timer"); break;
		case ESP_SLEEP_WAKEUP_TOUCHPAD  : Serial.println("Wakeup caused by touchpad"); break;
		case ESP_SLEEP_WAKEUP_ULP  : Serial.println("Wakeup caused by ULP program"); break;
        case ESP_SLEEP_WAKEUP_GPIO  : Serial.println("ESP_SLEEP_WAKEUP_GPIO"); break;
        case ESP_SLEEP_WAKEUP_UART  : Serial.println("ESP_SLEEP_WAKEUP_UART"); break;
		default : Serial.println("Wakeup was not caused by deep sleep"); break;
	}
}

// ESP_SLEEP_WAKEUP_UNDEFINED,    //!< In case of deep sleep, reset was not caused by exit from deep sleep
//     ESP_SLEEP_WAKEUP_ALL,          //!< Not a wakeup cause, used to disable all wakeup sources with esp_sleep_disable_wakeup_source
//     ESP_SLEEP_WAKEUP_EXT0,         //!< Wakeup caused by external signal using RTC_IO
//     ESP_SLEEP_WAKEUP_EXT1,         //!< Wakeup caused by external signal using RTC_CNTL
//     ESP_SLEEP_WAKEUP_TIMER,        //!< Wakeup caused by timer
//     ESP_SLEEP_WAKEUP_TOUCHPAD,     //!< Wakeup caused by touchpad
//     ESP_SLEEP_WAKEUP_ULP,          //!< Wakeup caused by ULP program
//     ESP_SLEEP_WAKEUP_GPIO,         //!< Wakeup caused by GPIO (light sleep only)
//     ESP_SLEEP_WAKEUP_UART,         //!< Wake


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

