/*
   HelTec Automation(TM) Low_Power test code,
   modified by DB.
*/

//#define DEPLOYING

#include "heltec.h"
//#include <ESP32_LoRaWAN.h> // didn't work so easy.
#include "Arduino.h"
#include <TinyGPS++.h>
#include <TimeLib.h> // 'system time' library.


//--------------------
// sleep variables
//--------------------
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define BAND    868E6  //you can set band here directly,e.g. 868E6,915E6
#ifdef DEPLOYING
#define SLEEP_TIMER1MIN 60        /* Time ESP32 will go to sleep (in seconds) */
#define SLEEP_TIMER3MIN 60*3        /* Time ESP32 will go to sleep (in seconds) */
#define SLEEP_TIMER15MIN 60*15        /* Time ESP32 will go to sleep (in seconds) */
#else
#define SLEEP_TIMER1MIN 3        /* Time ESP32 will go to sleep (in seconds) */
#define SLEEP_TIMER3MIN 2*3        /* Time ESP32 will go to sleep (in seconds) */
#define SLEEP_TIMER15MIN 1*15        /* Time ESP32 will go to sleep (in seconds) */
#endif

//--------------------
// gps variables
//--------------------
#define GPSBaud 9600
#define GPSPin 4
TinyGPSPlus gps; // create TinyGPS+ object
static const double ASTRAL_LAT = 53.145310, ASTRAL_LON = -4.115818;
double distanceRadiusKm = 0.000;


//------------------------------------------------------------
// RTC 'fast' memory variables (deep-sleep preserved)
//------------------------------------------------------------
RTC_DATA_ATTR int wakeup_reason_store;
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR bool firstBoot = true; // this is to know whether we're 'initialising' or 'running'.
RTC_DATA_ATTR bool awokenByVibration = false; // yet another flag
RTC_DATA_ATTR time_t tMem = 0;
typedef struct {
  uint32_t hdopStore;
  double longStore;
  double latStore;
} locationRecord;

RTC_DATA_ATTR locationRecord locRecord;


//--------------------------
// battery sense variables
//--------------------------
#define battADCPin 39
RTC_DATA_ATTR float battVoltage;


//--------------------------
// SETUP
//--------------------------
void setup() {
  Serial.begin(115200); // open debug serial port.
  Serial.println("Program Start");
  Serial1.begin(GPSBaud); // open GPS module serial port.

  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  print_wakeup_reason();
  
  //battery read initialising
  pinMode(battADCPin, INPUT);
  analogReadResolution(12);
  analogSetCycles(101); // seems optimum for our 220k/100k resistor divider.
  analogSetClockDiv(1);
  analogSetSamples(1);
  analogSetPinAttenuation(battADCPin, ADC_6db);
  Serial.print("batt voltage test : ");
  Serial.println(get_batt_voltage());

  // tinyGPS++ library example print.
  Serial.println();
  Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));



  /*
    delay(2000);
    Serial.begin(115200);
    //  setTime(1,1,1,1,1,2011);
    if (!timeStatus()) {
    Serial.println(tMem);
    }
    while(1);
  */


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
    esp_sleep_enable_timer_wakeup(SLEEP_TIMER3MIN * uS_TO_S_FACTOR); // 3 minute sleep to allow for GPS lock.
    pre_sleep();
    sleep_now();

  }

  //---------------------------------------------------
  // if awoken by the internal timer on first boot.
  //---------------------------------------------------
  if (wakeup_reason_store == 4 && firstBoot == true) {
    Serial.println("(wakeup_reason_store == 4 && firstBoot == true)");
    smartDelay(1000); // ensure we have fresh data input from the GPS module.
    gpsDebug();
    if (!gps.date.isValid() || !gps.location.isValid()) {
      Serial.println("gps not valid");
      pinMode(GPSPin, OUTPUT);
      digitalWrite(GPSPin, HIGH); // make sure gps is turned on?
      //      Serial.println(digitalRead(GPSPin));
      //      delay(500);
      // esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,0); //1 = High, 0 = Low
      esp_sleep_enable_timer_wakeup(SLEEP_TIMER1MIN * uS_TO_S_FACTOR);
      pre_sleep();
      sleep_now();
    }
    else { // we have valid data. So send packet and go to sleep.
      setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year()); // update system time.
      tMem = now(); // update RTC memory time.
      locRecord.hdopStore = gps.hdop.value(); // update RTC memory GPS data.
      locRecord.longStore = gps.location.lng(); // update RTC memory GPS data.
      locRecord.latStore = gps.location.lat(); // update RTC memory GPS data.
      send_LoRa();
      pinMode(GPSPin, OUTPUT);
      digitalWrite(GPSPin, LOW); Serial.println("gps turned off"); // turn off GPS.
      firstBoot = false;
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 0); Serial.println("vibration trig. activated"); //1 = wake on High, 0 = wake on Low
//      pre_sleep();
//      sleep_now();
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
    smartDelay(1000); // ensure we have fresh data input from the GPS module.
    gpsDebug();
    if (!gps.date.isValid() || !gps.location.isValid()) {
      Serial.println("gps not valid");
      // go to sleep on timer.
      esp_sleep_enable_timer_wakeup(SLEEP_TIMER1MIN * uS_TO_S_FACTOR);
      pre_sleep();
      sleep_now();
    }
    else {
      // send packet and sleep for 15 minutes minimum.
      setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
      if (worth_sending_check()) {
        send_LoRa();
        tMem = now(); // update RTC memory time.
        locRecord.hdopStore = gps.hdop.value();
        locRecord.longStore = gps.location.lng();
        locRecord.latStore = gps.location.lat();
        Serial.println("batt tbd");
        pinMode(GPSPin, OUTPUT);
        digitalWrite(GPSPin, LOW); Serial.println("gps turned off"); // turn off GPS
        awokenByVibration = false;
        esp_sleep_enable_timer_wakeup(SLEEP_TIMER15MIN * uS_TO_S_FACTOR);
        pre_sleep();
        sleep_now();
      }
      else { // if data isn't worth sending turn off gps and go to sleep 15 minutes.
        pinMode(GPSPin, OUTPUT);
        digitalWrite(GPSPin, LOW); Serial.println("gps turned off"); // turn off GPS
        awokenByVibration = false;
        esp_sleep_enable_timer_wakeup(SLEEP_TIMER15MIN * uS_TO_S_FACTOR);
        pre_sleep();
        sleep_now();
      }
    }
    battVoltage = get_batt_voltage();
    Serial.print("battVoltage: "); Serial.println(battVoltage);
  }


  //--------------------------------------------------------------------------------------------------------------
  // if awoken by the internal timer AFTER first boot, within a "awokenByVibration" cycle.
  //--------------------------------------------------------------------------------------------------------------
  if (wakeup_reason_store == 4 && firstBoot == false && awokenByVibration == true) {
    Serial.println("(wakeup_reason_store == 4 && firstBoot == false && awokenByVibration == true)");
    smartDelay(1000); // ensure we have fresh data string from the GPS module.
    gpsDebug();
    if (!gps.date.isValid() || !gps.location.isValid()) {
      Serial.println("gps not valid");
      pinMode(GPSPin, OUTPUT);
      digitalWrite(GPSPin, HIGH); Serial.println("gps turned on");
      // go straight back to sleep for one minute.
      esp_sleep_enable_timer_wakeup(SLEEP_TIMER1MIN * uS_TO_S_FACTOR);
      pre_sleep();
      sleep_now();
    }
    else { // we have valid data. So send packet and go to sleep.
      setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year()); // update system time.
      if (worth_sending_check()) {
        send_LoRa();
        tMem = now(); // update RTC memory time.
        locRecord.hdopStore = gps.hdop.value();
        locRecord.longStore = gps.location.lng();
        locRecord.latStore = gps.location.lat();
        Serial.println("batt tbd");
        pinMode(GPSPin, OUTPUT);
        digitalWrite(GPSPin, LOW); Serial.println("gps turned off");
        awokenByVibration = false;
        esp_sleep_enable_timer_wakeup(SLEEP_TIMER15MIN * uS_TO_S_FACTOR);
        pre_sleep();
        sleep_now();
      }
      else { // if data isn't worth sending turn off gps and go to sleep 15 minutes.
        pinMode(GPSPin, OUTPUT);
        digitalWrite(GPSPin, LOW); Serial.println("gps turned off");
        awokenByVibration = false;
        esp_sleep_enable_timer_wakeup(SLEEP_TIMER15MIN * uS_TO_S_FACTOR);
        pre_sleep();
        sleep_now();
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
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 0); Serial.println("vibration trig. activated"); //1 = wake on High, 0 = wake on Low
    pre_sleep();
    sleep_now();
  }



  /*
    Next we decide what all peripherals to shut down/keep on
    By default, ESP32 will automatically power down the peripherals
    not needed by the wakeup source, but if you want to be a poweruser
    this is for you. Read in detail at the API docs
    http://esp-idf.readthedocs.io/en/latest/api-reference/system/deep_sleep.html
    Left the line commented as an example of how to configure peripherals.
    The line below turns off all RTC peripherals in deep sleep.
  */
  //  esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  //  Serial.println("Configured all RTC Peripherals to be powered down in sleep");


}


void loop() {

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

void send_LoRa() {
  delay(30);
  Heltec.begin(false /*DisplayEnable Enable*/, true /*LoRa Disable*/, true /*Serial Enable*/, false /*PABOOST Enable*/, BAND /*long BAND*/);
  Serial.println("this send_LoRa() code needs doing");
}


/*
    Now that we have setup a wake cause and if needed setup the
    peripherals state in deep sleep, we can now start going to
    deep sleep.
    In the case that no wake up sources were provided but deep
    sleep was started, it will sleep forever unless hardware
    reset occurs.
*/
void pre_sleep() {
  LoRa.end();
  LoRa.sleep();
  //  delay(100);
  pinMode(5, INPUT);
  pinMode(14, INPUT);
  pinMode(15, INPUT);
  pinMode(16, INPUT);
  pinMode(17, INPUT);
  pinMode(18, INPUT);
  pinMode(19, INPUT);
  pinMode(26, INPUT);
  pinMode(27, INPUT);
  //  delay(100);
}

void sleep_now() {
  Serial.println("Going to sleep now");
  delay(2);
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
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

double get_batt_voltage() {
  long batt_accumulator = 0;
  for (int i = 0; i<10;i++) {
    batt_accumulator += analogRead(battADCPin);
  }

  double result = batt_accumulator / 10.0;
  return result;
}
