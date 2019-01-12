#include <Arduino.h>

int m0=12;
int m1=14;
int aux=13;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
  pinMode(m0,OUTPUT);
  pinMode(m1,OUTPUT);
  pinMode(aux,INPUT);
}

void loop() {
  digitalWrite(m0, 1);
  digitalWrite(m1,1);
  int mes[] = {0xc1,0xc1,0xc1};
  Serial2.write((uint8_t *)mes,3);
  delay(50);
  for (uint8_t n=0;n<6;n++)
  {
    Serial.print(Serial2.read());
    Serial.print(" ");
  }
  Serial.println("");
  delay(1000);
}
