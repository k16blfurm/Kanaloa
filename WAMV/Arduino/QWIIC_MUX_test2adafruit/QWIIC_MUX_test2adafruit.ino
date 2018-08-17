/**************************************************************************/
/*! 
    @file     trianglewave.pde
    @author   Adafruit Industries
    @license  BSD (see license.txt)
    This example will generate a triangle wave with the MCP4725 DAC.   
    This is an example sketch for the Adafruit MCP4725 breakout board
    ----> http://www.adafruit.com/products/935
 
    Adafruit invests time and resources providing this open source code, 
    please support Adafruit and open-source hardware by purchasing 
    products from Adafruit!
*/
/**************************************************************************/
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#define TCAADDR 0x70


Adafruit_MCP4725 dac;

void setup(void) {
  Serial.begin(9600);
  Serial.println("Hello!");

  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
    
  Serial.println("Generating a triangle wave");
}
//4095 is max
void loop(void) {
     uint32_t counter;
     dac.begin(0x62);
    
     tcaselect(1);
     counter= 3072; // Sets voltage to 5.0V * (0.75) [You do the math here]
     dac.setVoltage(counter, true); //Setting voltage for DAC 1
   
    tcaselect(5);
    counter=1024; // Sets voltage to 5.0V * (0.25)
    dac.setVoltage(counter, true);
/*
    tcaselect(3);
    counter= 3500; //Sets voltage to 5.0V * (3500/4096)
    dac.setVoltage(counter, true);
*/
    tcaselect(7);
    counter = 0; //Sets voltage to 5.0V * (0/4096);
    dac.setVoltage(counter, true);
 }

  
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
