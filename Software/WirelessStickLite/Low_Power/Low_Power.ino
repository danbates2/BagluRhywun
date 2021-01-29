/*
   HelTec Automation(TM) Low_Power test code,
   modified by DB.
*/

#include "heltec.h"

// sleep variables
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10        /* Time ESP32 will go to sleep (in seconds) */
#define BAND    433E6  //you can set band here directly,e.g. 868E6,915E6
RTC_DATA_ATTR int bootCount = 0;

// gps variables
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps; // create TinyGPS+ object

int wakeup_reason_store;
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

void setup() {
  if (wakeup_reason_store == 0) {
    
    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH); // turn on GPS.

    Serial.begin(115200); // open debug serial port.
    Serial1.begin(GPSBaud); // open GPS module serial port.
  
  }
  //WIFI Kit series V1 not support Vext control
  Heltec.begin(false /*DisplayEnable Enable*/, false /*LoRa Disable*/, true /*Serial Enable*/, false /*PABOOST Enable*/, BAND /*long BAND*/);

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  delay(20);

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  /*
    First we configure the wake up source
    We set our ESP32 to wake up every 5 seconds
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,0); //1 = High, 0 = Low

  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
                 " Seconds");
  delay(2000);

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

  /*
    Now that we have setup a wake cause and if needed setup the
    peripherals state in deep sleep, we can now start going to
    deep sleep.
    In the case that no wake up sources were provided but deep
    sleep was started, it will sleep forever unless hardware
    reset occurs.
  */
  LoRa.end();
  LoRa.sleep();
  delay(100);

  pinMode(5, INPUT);
  pinMode(14, INPUT);
  pinMode(15, INPUT);
  pinMode(16, INPUT);
  pinMode(17, INPUT);
  pinMode(18, INPUT);
  pinMode(19, INPUT);
  pinMode(26, INPUT);
  pinMode(27, INPUT);

  delay(100);
  Serial.println("Going to sleep now");
  delay(2);
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}

void loop() {
  //This is not going to be called
}
