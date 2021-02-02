/*
   HelTec Automation(TM) LoRaWAN 1.0.2 OTAA example use OTAA, CLASS A

   Function summary:

   - use internal RTC(150KHz);

   - Forward the data measured by the 12-bit ADC to the server.

   - Include stop mode and deep sleep mode;

   - 15S data send cycle;

   - Informations output via serial(115200);

   - Only ESP32 + LoRa series boards can use this library, need a license
     to make the code run(check you license here: http://www.heltec.cn/search/);

   You can change some definition in "Commissioning.h" and "LoRaMac-definitions.h"

   HelTec AutoMation, Chengdu, China.
   成都惠利特自动化科技有限公司
   https://heltec.org
   support@heltec.cn

  this project also release in GitHub:
  https://github.com/HelTecAutomation/ESP32_LoRaWAN
*/

#include <ESP32_LoRaWAN.h>
#include <Mcu.h>
#include "Arduino.h"
#include <TinyGPS++.h> // gps library for neo-6mv2 module.
#include <TimeLib.h> // 'system time' library.

//#define DEPLOYING // this elongates the timers to the long enough times for battery saving. uncomment for testing and quicker turnovers of timing events.
// https://console.thethingsnetwork.org/applications/horsesforcourses/devices/danshorsetest

// ABP mode!

/*license for Heltec ESP32 LoRaWan, quary your ChipID relevant license: http://resource.heltec.cn/search */
uint32_t  license[4] = {0x06E006C1, 0xDF66D23C, 0xE96D5039, 0x4B477066};

/* OTAA para*/
uint8_t DevEui[] = { 0x00, 0x15, 0x50, 0xDB, 0x77, 0xD3, 0x05, 0xBA };
uint8_t AppEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x03, 0x51, 0x8C };
uint8_t AppKey[] = { 0xB1, 0x55, 0x3A, 0x32, 0x45, 0xE3, 0xD0, 0xDE, 0x2B, 0xB0, 0x4E, 0x07, 0x48, 0x05, 0x40, 0xA3 };

/* ABP para*/
uint8_t NwkSKey[] = { 0xF3, 0x13, 0x08, 0x75, 0x13, 0xE3, 0x9D, 0x60, 0xF5, 0x91, 0xA5, 0xC0, 0x65, 0x41, 0x88, 0xE0 };
uint8_t AppSKey[] = { 0xFC, 0x23, 0x6A, 0x57, 0x5D, 0x32, 0xD2, 0xE4, 0xC7, 0x31, 0x90, 0x9C, 0xC1, 0xD0, 0x43, 0x4B };
uint32_t DevAddr =  ( uint32_t )0x260133C9;

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 10000;

/*OTAA or ABP*/
bool overTheAirActivation = false;

/*ADR enable*/
bool loraWanAdr = false;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = false;

/* Application port */
uint8_t appPort = 2;

/*!
  Number of trials to transmit the frame, if the LoRaMAC layer did not
  receive an acknowledgment. The MAC performs a datarate adaptation,
  according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
  to the following table:

  Transmission nb | Data Rate
  ----------------|-----------
  1 (first)       | DR
  2               | DR
  3               | max(DR-1,0)
  4               | max(DR-1,0)
  5               | max(DR-2,0)
  6               | max(DR-2,0)
  7               | max(DR-3,0)
  8               | max(DR-3,0)

  Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
  the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 8;

/*LoraWan debug level, select in arduino IDE tools.
  None : print basic info.
  Freq : print Tx and Rx freq, DR info.
  Freq && DIO : print Tx and Rx freq, DR, DIO0 interrupt and DIO1 interrupt info.
  Freq && DIO && PW: print Tx and Rx freq, DR, DIO0 interrupt, DIO1 interrupt and MCU deepsleep info.
*/
uint8_t debugLevel = LoRaWAN_DEBUG_LEVEL;

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;


//----------------------------------------
// ready for sending bool
//----------------------------------------
bool dataisValidAndForSending = false;
RTC_DATA_ATTR bool firstBootSender = true;

//--------------------
// sleep variables
//--------------------
#define Vext 21
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define mS_TO_S_FACTOR 1000  /* Conversion factor for micro seconds to seconds */
//#define BAND 868E6  //you can set band here directly,e.g. 868E6,915E6
#ifdef DEPLOYING // these timers should ultimately end up in ms to work with 'uint32_t appTxDutyCycle'.
#define SLEEP_TIMER1MIN mS_TO_S_FACTOR * 60        /* Time ESP32 will go to sleep (in seconds) */
#define SLEEP_TIMER3MIN mS_TO_S_FACTOR * 60*3        /* Time ESP32 will go to sleep (in seconds) */
#define SLEEP_TIMER15MIN mS_TO_S_FACTOR * 60*15        /* Time ESP32 will go to sleep (in seconds) */
#else
#define SLEEP_TIMER1MIN mS_TO_S_FACTOR * 15  // seconds
#define SLEEP_TIMER3MIN mS_TO_S_FACTOR * 30
#define SLEEP_TIMER15MIN mS_TO_S_FACTOR * 60
#endif
#define UINT32MAX 2^32
bool DEVICE_STATE_SLEEP_BOOL = false; // for debugging and avoiding repetative SLEEP state print outs.

//--------------------
// gps variables
//--------------------
#define GPSBaud 9600
#define GPSPin 4
TinyGPSPlus gps; // create TinyGPS+ object
static const double ASTRAL_LAT = 53.145310, ASTRAL_LON = -4.115818;
float distanceRadiusKm = 0.020;


//------------------------------------------------------------
// RTC 'fast' memory variables (deep-sleep preserved)
//------------------------------------------------------------
RTC_DATA_ATTR int wakeup_reason_store;
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR bool firstBoot = true; // this is to know whether we're 'initialising' or 'running'.
RTC_DATA_ATTR bool awokenByVibration = false; // yet another flag
RTC_DATA_ATTR time_t tMem = 0;
typedef struct {
  float hdopStore = 101.0;
  float longStore = 0.1;
  float latStore = 0.1;
  float battVoltage = 0.1;
} locationRecord_t; // init'd with test data

RTC_DATA_ATTR locationRecord_t locRecord;
RTC_DATA_ATTR locationRecord_t loraPacketStore;



//--------------------------
// battery sense variables
//--------------------------
#define battADCPin 39
//RTC_DATA_ATTR float battVoltage; // moved to struct above, locationRecord;


// Add your initialization code here
void setup()
{
  // have to put Vext LOW on this one.
  Serial.begin(115200);
  while (!Serial);
  Serial.println("ABP to Deploy");
  Serial.print("Program LoRaWAN packet size : "); Serial.println(sizeof(locRecord));
  Serial1.begin(GPSBaud); // open GPS module serial port.

  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  print_wakeup_reason();

  SPI.begin(SCK, MISO, MOSI, SS);
  Mcu.init(SS, RST_LoRa, DIO0, DIO1, license);

  // battery read initialising
  pinMode(battADCPin, INPUT);
  analogReadResolution(12);
  analogSetCycles(101); // seems optimum for our 220k/100k resistor divider.
  analogSetClockDiv(1);
  analogSetSamples(1);
  analogSetPinAttenuation(battADCPin, ADC_6db);
  Serial.print("batt voltage test : ");
  Serial.println(get_batt_voltage());


  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH);





  //----------------------------
  // if booted from scratch
  //----------------------------
  if (wakeup_reason_store == 0) {
    //    Heltec.begin(false /*DisplayEnable Enable*/, true /*LoRa Disable*/, true /*Serial Enable*/, false /*PABOOST Enable*/, BAND /*long BAND*/);
    Serial.println("(wakeup_reason_store == 0)");
    pinMode(GPSPin, OUTPUT);
    digitalWrite(GPSPin, HIGH); Serial.println("gps turned on"); // turn on GPS.
    smartDelay(1000);
    gpsDebug();
    appTxDutyCycle = SLEEP_TIMER3MIN; Serial.println("Setting sleep timer to 3 minutes");
  }

  //---------------------------------------------------
  // if awoken by the internal timer on first boot.
  //---------------------------------------------------
  if (wakeup_reason_store == 4 && firstBoot == true) {
    Serial.println("(wakeup_reason_store == 4 && firstBoot == true)");
    smartDelay(1000); // ensure we have fresh data input from the GPS module.
    gpsDebug();
    if (gps.hdop.hdop() > 10.0 || gps.hdop.hdop() < 0.1) { //!gps.date.isValid() || !gps.location.isValid()
      Serial.println("gps not valid");
      pinMode(GPSPin, OUTPUT);
      digitalWrite(GPSPin, HIGH);// make sure gps is turned on?
      appTxDutyCycle = SLEEP_TIMER1MIN; Serial.println("Setting sleep timer to 1 minutes");
    }
    else { // we have valid data. So send packet and go to sleep.
      firstBoot = false;
      setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year()); // update system time.
      delay(2);
      rtc_data_updater(); // update rtc memory data with fresh gps and battery data.
      delay(2);
      lora_packet_preparer(); // copy gps and battery data to a struct for sending over LoRaWAN.
      delay(2);

      // tidy up.
      pinMode(GPSPin, OUTPUT);
      digitalWrite(GPSPin, LOW); Serial.println("gps turned off"); // turn off GPS.
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 0); Serial.println("vibration trig. activated"); // 1 = wake on High, 0 = wake on Low
      appTxDutyCycle = UINT32MAX; Serial.println("Setting sleep timer to UINT32MAX");
    }
  }

  //-----------------------------------
  // if awoken by a vibration
  //-----------------------------------
  if (wakeup_reason_store == 2) {
    Serial.println("AWOKEN BY VIBRATION!!!");
    awokenByVibration = true; // AWOKEN BY VIBRATION!!!
    pinMode(GPSPin, OUTPUT);
    digitalWrite(GPSPin, HIGH); Serial.println("gps turned on"); // turn on GPS.
    delay(5000); smartDelay(1000); // ensure we have fresh data input from the GPS module.
    gpsDebug();
    if (gps.hdop.hdop() > 10.0 || gps.hdop.hdop() < 0.1) {
      Serial.println("gps not valid");
      // go to sleep on timer.
      appTxDutyCycle = SLEEP_TIMER1MIN; Serial.println("Setting sleep timer to 1 minutes");
    }
    else {
      // send packet and sleep for 15 minutes minimum.

      setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
      if (worth_sending_check()) {
        lora_packet_preparer(); // copy gps and battery data to a struct for sending over LoRaWAN.
        rtc_data_updater(); // update rtc memory data with fresh gps and battery data.

        // tidy up.
        pinMode(GPSPin, OUTPUT);
        digitalWrite(GPSPin, LOW); Serial.println("gps turned off"); // turn off GPS
        awokenByVibration = false;
        appTxDutyCycle = SLEEP_TIMER15MIN; Serial.println("Setting sleep timer to 15 minutes");
      }
      else { // if data isn't worth sending turn off gps and go to sleep 15 minutes.
        pinMode(GPSPin, OUTPUT);
        digitalWrite(GPSPin, LOW); Serial.println("gps turned off"); // turn off GPS
        awokenByVibration = false;
        appTxDutyCycle = SLEEP_TIMER15MIN; Serial.println("Setting sleep timer to 15 minutes");
      }
    }
  }


  //--------------------------------------------------------------------------------------------------------------
  // if awoken by the internal timer AFTER first boot,
  // outside of an "awokenByVibration" cycle...
  // should be after a long sleep... waking ONLY to activate the vibration sensor wake interrupt.
  //--------------------------------------------------------------------------------------------------------------
  if (wakeup_reason_store == 4 && firstBoot == false && awokenByVibration == false) {
    Serial.println("(wakeup_reason_store == 4 && firstBoot == false && awokenByVibration == false)");
    pinMode(GPSPin, OUTPUT);
    digitalWrite(GPSPin, LOW); Serial.println("gps turned off"); // make sure GPS turned off.
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 0); Serial.println("vibration trig. activated"); // 1 = wake on High, 0 = wake on Low
    appTxDutyCycle = UINT32MAX; Serial.println("Setting sleep timer to UINT32MAX");

    // pre_sleep();
    //    sleep_now();
  }


  //--------------------------------------------------------------------------------------------------------------
  // if awoken by the internal timer AFTER first boot, within a "awokenByVibration" cycle.
  //--------------------------------------------------------------------------------------------------------------
  if (wakeup_reason_store == 4 && firstBoot == false && awokenByVibration == true) {
    Serial.println("(wakeup_reason_store == 4 && firstBoot == false && awokenByVibration == true)");
    smartDelay(1000); // ensure we have fresh data string from the GPS module.
    gpsDebug();
    if (gps.hdop.hdop() > 10.0 || gps.hdop.hdop() < 0.1) {
      Serial.println("gps not valid");
      pinMode(GPSPin, OUTPUT);
      digitalWrite(GPSPin, HIGH); Serial.println("gps turned on");
      // go straight back to sleep for one minute.
      appTxDutyCycle = SLEEP_TIMER1MIN; Serial.println("Setting sleep timer to 1 minutes");
    }
    else { // we have valid data. So send packet and go to sleep.
      setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year()); // update system time.
      if (worth_sending_check()) {
        lora_packet_preparer(); // copy gps and battery data to a struct for sending over LoRaWAN.
        rtc_data_updater(); // update rtc memory data with fresh gps and battery data.

        // tidy up
        pinMode(GPSPin, OUTPUT);
        digitalWrite(GPSPin, LOW); Serial.println("gps turned off");
        awokenByVibration = false;
        appTxDutyCycle = SLEEP_TIMER15MIN; Serial.println("Setting sleep timer to 15 minutes");
      }
      else { // if data isn't worth sending turn off gps and go to sleep 15 minutes.
        pinMode(GPSPin, OUTPUT);
        digitalWrite(GPSPin, LOW); Serial.println("gps turned off");
        awokenByVibration = false;
        appTxDutyCycle = SLEEP_TIMER15MIN; Serial.println("Setting sleep timer to 15 minutes");
      }
    }
  }






  deviceState = DEVICE_STATE_INIT;

  Serial.println("setup() end");
}





// The loop function is called in an endless loop
void loop()
{
  switch ( deviceState )
  {
    case DEVICE_STATE_INIT:
      {
        Serial.println("DEVICE_STATE_INIT");
        if (DEVICE_STATE_SLEEP_BOOL) {
          DEVICE_STATE_SLEEP_BOOL = false;
        }
        LoRaWAN.init(loraWanClass, loraWanRegion);
        break;
      }
    case DEVICE_STATE_JOIN:
      {
        Serial.println("DEVICE_STATE_JOIN");
        if (DEVICE_STATE_SLEEP_BOOL) {
          DEVICE_STATE_SLEEP_BOOL = false;
        }
        LoRaWAN.join();
        break;
      }
    case DEVICE_STATE_SEND:
      {
        Serial.println("DEVICE_STATE_SEND");
        if (DEVICE_STATE_SLEEP_BOOL) {
          DEVICE_STATE_SLEEP_BOOL = false;
        }

        if (firstBootSender) {
          Serial.println("first Boot sending");

          // clear flag
          firstBootSender = false;

          // bring gps and battery data together into LoRa packet.
          bring_data_together();

          // send packet.
          LoRaWAN.send(loraWanClass);
        }

        if (dataisValidAndForSending) {
          // clear flag
          dataisValidAndForSending = false;

          // bring gps and battery data together into LoRa packet.
          bring_data_together();
          delay(2);

          // send packet.
          LoRaWAN.send(loraWanClass);
        }


        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        Serial.println("DEVICE_STATE_CYCLE");
        if (DEVICE_STATE_SLEEP_BOOL) {
          DEVICE_STATE_SLEEP_BOOL = false;
        }
        // Schedule next packet transmission
        txDutyCycleTime = appTxDutyCycle + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
        Serial.println(txDutyCycleTime);
        LoRaWAN.cycle(txDutyCycleTime);
        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        if (!DEVICE_STATE_SLEEP_BOOL) { // for debugging and avoiding SLEEP state print outs.
          DEVICE_STATE_SLEEP_BOOL = true;
          Serial.println("DEVICE_STATE_SLEEP");
        }
        LoRaWAN.sleep(loraWanClass, debugLevel);
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }
}


bool worth_sending_check() {
  Serial.println("worth_sending_check");

  bool timeChecker;
  Serial.print("now() - tMem = ");
  Serial.println(now() - tMem);
  if (now() - tMem > (60 * 15)) { // if time between now and previous packet send greater than 15 minutes.
    Serial.println("timeChecker is true");
    timeChecker = true;
  }
  else {
    Serial.println("timeChecker is false");
    timeChecker = false;
  }

  bool distanceChecker;
  if (distanceResult(gps.location.lat(), gps.location.lng(), locRecord.latStore, locRecord.longStore) > distanceRadiusKm) {
    distanceChecker = true;
    Serial.println("distanceChecker is true");
  }
  else {
    Serial.println("distanceChecker is false");
    distanceChecker = false;
  }

  // and if either are met...
  if (timeChecker || distanceChecker) {
    Serial.println("worth sending yes");
    return true;
  }
  else {
    Serial.println("worth sending No");
    return false;
  }
}

double distanceResult(double lat_now, double lng_now, double lat_previous, double lng_previous) {
  Serial.println("distanceResult");
  double distanceKm = TinyGPSPlus::distanceBetween(
                        lat_now,
                        lng_now,
                        lat_previous,
                        lng_previous) / 1000;
  Serial.println(distanceKm);
  return distanceKm;
}


/*
  Method to print the reason by which ESP32
  has been awaken from sleep
*/
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
    case 1  :
      {
        Serial.println("Wakeup caused by external signal using RTC_IO");
        wakeup_reason_store = 1;
        delay(2);
      } break;
    case 2  :
      {
        Serial.println("Wakeup caused by external signal using RTC_CNTL");
        wakeup_reason_store = 2;
        delay(2);
      } break;
    case 3  :
      {
        Serial.println("Wakeup caused by timer");
        wakeup_reason_store = 3;
        delay(2);
      } break;
    case 4  :
      {
        Serial.println("Wakeup caused by touchpad");
        wakeup_reason_store = 4;
        delay(2);
      } break;
    case 5  :
      {
        Serial.println("Wakeup caused by ULP program");
        wakeup_reason_store = 5;
        delay(2);
      } break;
    default :
      {
        Serial.println("Wakeup was not caused by deep sleep");
        wakeup_reason_store = 0;
        delay(2);
      } break;
  }
}


// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate & d, TinyGPSTime & t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }

  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i = 0; i < len; ++i)
    Serial.print(i < slen ? str[i] : ' ');
  smartDelay(0);
}

void debugExit() {
  Serial.println("debugExit()");
}

void gpsDebug() {

  // tinyGPS++ library example print.
  Serial.println();
  Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));


  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

  unsigned long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT,
      LONDON_LON) / 1000;
  printInt(distanceKmToLondon, gps.location.isValid(), 9);

  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT,
      LONDON_LON);

  printFloat(courseToLondon, gps.location.isValid(), 7, 2);

  const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

  printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();

  //  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));

}

float get_batt_voltage() {
  long batt_accumulator = 0;
  for (int i = 0; i < 10; i++) {
    batt_accumulator += analogRead(battADCPin);
  }

  double result = batt_accumulator / 10.0;
  return result;
}


// https://stackoverflow.com/questions/484357/trying-to-copy-struct-members-to-byte-array-in-c
// Transfer data from our struct to the LoRa packet, the packet is called appData. appDataSize must also be defined.
void bring_data_together() {
  Serial.println("loraPacketStore :");
  Serial.print("packing data into LoRa packet : ");
  uint8_t *bytePtr = (uint8_t*)&loraPacketStore;
  appDataSize = sizeof(loraPacketStore) ;

  // appData MAX length is 128 bytes.

  for (int i = 0; i < sizeof(loraPacketStore); i++) {
    appData[i] = *bytePtr;
    bytePtr++;
    Serial.print(appData[i], HEX);
  }
  Serial.println();
}

void lora_packet_preparer() {
  Serial.println("lora_packet_preparer() : ");
  
  loraPacketStore.hdopStore = gps.hdop.hdop();
  Serial.println(loraPacketStore.hdopStore);

  loraPacketStore.latStore = (float)gps.location.lat();
  Serial.println(loraPacketStore.latStore);
  
  loraPacketStore.longStore = (float)gps.location.lng();
  Serial.println(loraPacketStore.longStore);
  
  loraPacketStore.battVoltage = get_batt_voltage();
  Serial.println(loraPacketStore.battVoltage);

  dataisValidAndForSending = true;
}

void rtc_data_updater() {
  // set the system time with the new gps time.
  Serial.println("rtc_data_updater()"); 
  // update RTC memory time with new system time..
  tMem = now();
  // save the new gps and battery data.
  locRecord.hdopStore = gps.hdop.hdop(); // update RTC memory GPS data.
  locRecord.longStore = gps.location.lng(); // update RTC memory GPS data.
  locRecord.latStore = gps.location.lat(); // update RTC memory GPS data.
  locRecord.battVoltage = get_batt_voltage();
  Serial.print("battVoltage: "); Serial.println(locRecord.battVoltage);
}
