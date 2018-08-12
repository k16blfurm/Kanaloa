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
     enableMuxPort(1); //Opens Mux Port 1
     counter= 3072; // Sets voltage to 3.3V * (0.75) [You do the math here]
     dac.setVoltage(counter, true); //Setting voltage for DAC 1
     disableMuxPort(1); // Closes Port 1
    enableMuxPort(2);
    counter=1024; // Sets voltage to 3.3V * (0.25)
    dac.setVoltage(counter, true);
    disableMuxPort(2);
 }
