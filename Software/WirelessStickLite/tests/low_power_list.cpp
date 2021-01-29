//-----------------------
// from Low_Power.ino
//-----------------------
#define Vext 21

  LoRa.end();
  LoRa.sleep();
  delay(100);
  pinMode(5,INPUT);
  pinMode(14,INPUT);
  pinMode(15,INPUT);
  pinMode(16,INPUT);
  pinMode(17,INPUT);
  pinMode(18,INPUT);
  pinMode(19,INPUT);
  pinMode(26,INPUT);
  pinMode(27,INPUT);
  delay(2);
  digitalWrite(Vext,HIGH);

//-----------------------
// from heltec.cpp
//-----------------------
void Heltec_ESP32::VextON(void) // called at beginning of setup().
{
	pinMode(Vext,OUTPUT);
	digitalWrite(Vext, LOW);
}

void Heltec_ESP32::VextOFF(void) // Vext default OFF for low power.
{
	pinMode(Vext,OUTPUT);
	digitalWrite(Vext, HIGH);
}



//--------------------
// from LoRa.cpp
//--------------------
void LoRaClass::end()
{
  // put in sleep mode
  sleep();
  // stop SPI
  SPI.end();
}


void LoRaClass::sleep()
{
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}
