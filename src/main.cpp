#include <Arduino.h>
#include "E32Lora.h"

int m0=12;
int m1=14;
int aux=13;

E32Lora e(Serial2);
void setup() {
  Serial.begin(115200);
  Serial.println("Setup");

  e.begin(m0,m1,aux);
  e.setTargetAddress(0x02);
  e.setTargetChannel(0x0f);
  e.setMode(NORMAL_MODE);
  e.reset();
}

void loop() {

  //Serial.println("Loop");
  uint8_t config[512];
  //e.getConfig(config);
  //Serial.println("Config ->");

//  if(e.dataAvailable()) {
//    Serial.println("Data");
//    int16_t dataBytes = e.receiveData(config, 512);
//    Serial.println(dataBytes);

//    for (uint8_t n=0;n<dataBytes+1;n+=10) {
//      Serial.print(config[n],HEX);
//      Serial.print(" ");
//    }
//    Serial.println("");
//    e.getConfig(config);
//    Serial.println("Config ->");
//    for (uint8_t n=0;n<6;n++) {
//      Serial.print(config[n],HEX);
//      Serial.print(" ");
//    }

//    e.setMode(SLEEP_MODE);
//    delay(100);
//    e.setMode(NORMAL_MODE);
//    delay(100);

//    Serial.println("");
//    Serial.println("");
//  }
  uint8_t mes[] = {1,2,3};
  E32_STATUS status = e.transmit(mes,3);
  Serial.println(status);
  delay(1000);
//  while (Serial2.available()){
//    Serial.print(Serial2.read(),HEX);
//    Serial.print(" ");
//  }
//  e.getConfig(config);
//  Serial.println("Config ->");
//  for (uint8_t n=0;n<6;n++) {
//    Serial.print(config[n],HEX);
//    Serial.print(" ");
//  }
//  delay(1000);
  //Serial.println("");

  //uint8_t mes[] = {1,2,3};
  //e.setTargetAddress(0x00);
  //e.setTargetChannel(0x0f);
  //E32_STATUS s = e.transmit(mes,3);
  //Serial.print("Status ->");
  //Serial.println(s);
  //uint8_t mes[] = {0xc1,0xc1,0xc1};
  //hs.write(mes,3);
  //delay(50);
  //for (uint8_t n=0;n<6;n++)
  //{
  //  Serial.print(hs.read(),HEX);
  //  Serial.print(" ");
  //}
  //Serial.println("");

}
