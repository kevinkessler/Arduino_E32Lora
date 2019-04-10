#include <Arduino.h>
#include "E32Lora.h"

int m0=12;
int m1=14;
int aux=13;

E32Lora e(Serial2);
void setup() {
  Serial.begin(115200);
  e.begin(m0,m1,aux);
  e.setMode(CONFIG_MODE);
  e.setTransmissionMode(TxMode_Fixed);
  e.setAddress(0x00);
  e.setChannel(0xf);
  e.setTargetAddress(0x02);
  e.setTargetChannel(0x0f);
  e.saveParams();
  e.setMode(NORMAL_MODE); 
  e.reset();
}

void loop() {

  E32_STATUS status = e.getConfig(config);
  for(int n=0;n<6;n++)
  {
    Serial.print(config[n],HEX);
    Serial.print(" ");
  }
  Serial.println("");


  if(e.dataAvailable()) {
    uint8_t databytes[100];
    Serial.println("Data");
    int16_t dataBytes = e.receiveData(databytes,100);
    Serial.println(dataBytes);

    for (uint8_t n=0;n<dataBytes+1;n++) {
      Serial.print(config[n],HEX);
      Serial.print(" ");
    }
  }

  uint8_t mes[] = {1,2,3};
  status = e.transmit(mes,3);
  Serial.println(status);
  delay(1000);

}
