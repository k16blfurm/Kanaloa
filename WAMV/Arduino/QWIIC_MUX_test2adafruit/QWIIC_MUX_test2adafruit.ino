#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
Adafruit_MCP4725 dac;

void setup(void) {
  Serial.begin(9600);
   dac.begin(0x62);
}
//4095 is max
void loop(void) {
  for (int counter=0;counter<4095;counter++){
    individualset(counter,counter,counter,counter);
    delay(10);
  }
}










void tcaselect(uint8_t i) {
  //Opens Ports on the multiplexer to communicate with individual DACs
  //i is the number of the port
  //Provided by Adafruit Documentation
  if (i > 7) return;
  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void individualset(int Q1,int Q2,int Q3,int Q4){
  tcaselect(0);
  dac.setVoltage(Q1,false);
  tcaselect(1);
  dac.setVoltage(Q2,false);
  tcaselect(2);
  dac.setVoltage(Q3,false);
  tcaselect(3);
  dac.setVoltage(Q4,false);
  Serial.println("I went through!");
}
