/*
 * HelTec Automation(TM) LoRaWAN 1.0.2 OTAA example use OTAA, CLASS A
 *
 * Function summary:
 *
 * - use internal RTC(150KHz);
 *
 * - Forward the data measured by the 12-bit ADC to the server.
 *
 * - Include stop mode and deep sleep mode;
 *
 * - 15S data send cycle;
 *
 * - Informations output via serial(115200);
 *
 * - Only ESP32 + LoRa series boards can use this library, need a license
 *   to make the code run(check you license here: http://www.heltec.cn/search/);
 *
 * You can change some definition in "Commissioning.h" and "LoRaMac-definitions.h"
 *
 * HelTec AutoMation, Chengdu, China.
 * 成都惠利特自动化科技有限公司
 * https://heltec.org
 * support@heltec.cn
 *
 *this project also release in GitHub:
 *https://github.com/HelTecAutomation/ESP32_LoRaWAN
*/

#include <ESP32_LoRaWAN.h>
#include "Arduino.h"

/*license for Heltec ESP32 LoRaWan, quary your ChipID relevant license: http://resource.heltec.cn/search */
uint32_t  license[4] = {0x06E006C1,0xDF66D23C,0xE96D5039,0x4B477066};

/* OTAA para*/
uint8_t DevEui[] = { 0x00, 0x7C, 0x66, 0xAA, 0x00, 0xA6, 0x20, 0xFB };
uint8_t AppEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x03, 0x51, 0x8C };
uint8_t AppKey[] = { 0xB1, 0x55, 0x3A, 0x32, 0x45, 0xE3, 0xD0, 0xDE, 0x2B, 0xB0, 0x4E, 0x07, 0x48, 0x05, 0x40, 0xA3 };

/* ABP para*/
uint8_t NwkSKey[] = { 0xD5, 0xB4, 0x4E, 0x8A, 0x33, 0x10, 0xB5, 0x0C, 0xCE, 0x8B, 0x5E, 0xEE, 0x6E, 0x45, 0x9E, 0xEF };
uint8_t AppSKey[] = { 0xDF, 0xD6, 0xED, 0xFA, 0x93, 0x0C, 0xFE, 0xA4, 0x98, 0xD7, 0x0E, 0x13, 0x44, 0x10, 0xE1, 0x65 };
uint32_t DevAddr =  ( uint32_t )0x26015F02;

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 15000;

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;

/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 8;

/*LoraWan debug level, select in arduino IDE tools.
* None : print basic info.
* Freq : print Tx and Rx freq, DR info.
* Freq && DIO : print Tx and Rx freq, DR, DIO0 interrupt and DIO1 interrupt info.
* Freq && DIO && PW: print Tx and Rx freq, DR, DIO0 interrupt, DIO1 interrupt and MCU deepsleep info.
*/
uint8_t debugLevel = LoRaWAN_DEBUG_LEVEL;

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

static void ADC_Process(uint16_t val){
    appDataSize = 4 ;
    int TH     = (int)(val / 1000);
    appData[3] = (char)(TH) ;

    int HU     = (int)((val - TH*1000) / 100);
    appData[2] = (char)(HU) ;

    int Ten    = (int)((val - TH * 1000 - HU * 100) / 10);
    appData[1] = (char)(Ten) ;

    int Sin    = (val % 10);
    appData[0] = (char)(Sin) ;
}
#define Vext 21
// Add your initialization code here
void setup()
{
  Serial.begin(115200);
  while (!Serial);
  SPI.begin(SCK,MISO,MOSI,SS);
  Mcu.init(SS,RST_LoRa,DIO0,DIO1,license);
  
  adcAttachPin(39);
  analogSetClockDiv(255); // 1338mS
    pinMode(Vext, OUTPUT);  
  deviceState = DEVICE_STATE_INIT;
}

// The loop function is called in an endless loop
void loop()
{
  switch( deviceState )
  {
    case DEVICE_STATE_INIT:
    {
      LoRaWAN.init(loraWanClass,loraWanRegion);
      break;
    }
    case DEVICE_STATE_JOIN:
    {
      LoRaWAN.join();
      break;
    }
    case DEVICE_STATE_SEND:
    {
      digitalWrite(Vext, LOW);
      delay(10);
      uint16_t ADC_voltage = analogRead(37);
      digitalWrite(Vext, HIGH);
      ADC_Process( ADC_voltage );
      LoRaWAN.send(loraWanClass);
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }
    case DEVICE_STATE_CYCLE:
    {
      // Schedule next packet transmission
      txDutyCycleTime = appTxDutyCycle + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP:
    {
      LoRaWAN.sleep(loraWanClass,debugLevel);
      break;
    }
    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}
