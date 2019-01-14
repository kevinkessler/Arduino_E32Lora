#include <Arduino.h>
#include "E32Lora.h"

int m0=12;
int m1=14;
int aux=13;
//HardwareSerial hs(1);
E32Lora e(Serial2, m0, m1, aux);
void setup() {
  Serial.begin(115200);
  Serial.println("Setup");
  pinMode(m0,OUTPUT);
  pinMode(m1,OUTPUT);
  pinMode(aux,INPUT);
}

void loop() {

  Serial.println("Loop");
  uint8_t config[6];
  e.getConfig(config);
  Serial.println("Config ->");
  for (uint8_t n=0;n<6;n++) {
    Serial.print(config[n],HEX);
    Serial.print(" ");
  }

  Serial.println("");
  e.getVersion(config);
  Serial.println("Version ->");
  for (uint8_t n=0;n<6;n++) {
    Serial.print(config[n],HEX);
    Serial.print(" ");
  }

  Serial.println("");

  uint8_t mes[] = {1,2,3};
  e.setTargetAddress(0x00);
  e.setTargetChannel(0x0f);
  E32_STATUS s = e.transmit(mes,3);
  Serial.print("Status ->");
  Serial.println(s);
  //uint8_t mes[] = {0xc1,0xc1,0xc1};
  //hs.write(mes,3);
  //delay(50);
  //for (uint8_t n=0;n<6;n++)
  //{
  //  Serial.print(hs.read(),HEX);
  //  Serial.print(" ");
  //}
  Serial.println("");

  delay(1000);
}
