#define adcPin 39

void setup() {
  
  // put your setup code here, to run once:
  delay(2000);
  Serial.begin(115200);
  Serial.println("go");
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  pinMode(adcPin, INPUT);
  delay(10);
  analogReadResolution(12);
  analogSetCycles(101); // seems optimum for our 220k/100k resistor divider.
  analogSetClockDiv(1);
  analogSetSamples(1);
  analogSetPinAttenuation(adcPin, ADC_6db);
//  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_0db);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = 0; i<255; i++) {
    analogSetCycles(i);
    Serial.print("analogCycles: "); Serial.print(i);
    Serial.print(", reading: "); Serial.println(analogRead(adcPin));
//    delay(50);
  }
//  Serial.println(analogRead(adcPin));
  delay(1000);

}
