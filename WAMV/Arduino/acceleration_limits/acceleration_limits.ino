#include <Adafruit_MCP4725.h> //For DACs
Adafruit_MCP4725 dac; //just a constructor
//Definitions
#define TCAADDR 0x70

void tcaselect(uint8_t i) {
  //Opens Ports on the multiplexer to communicate with individual DACs
  //i is the number of the port
  //Provided by Adafruit Documentation
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void accellimit(int Q1L,int Q2L,int Q3L,int Q4L,int Q1Out,int Q2Out,int Q3Out,int Q4Out)
{
  //This program prevents sudden change in voltage to the motor controllers by slowly incrementing the voltage sent to 
  //the motor controllers to the desired voltage
  //Initiate I2c Communication with DACs
     dac.begin(0x62);
  const bool forward=true;
  const bool backward=false;
  bool Q1Dir;
  bool Q2Dir;
  bool Q3Dir;
  bool Q4Dir;
  //Determine which motor is decelerating
  Q1Dir = (Q1Out-Q1L < 0)?backward:forward; //ternary operators Basically a shorthand if, else statement;
  Q2Dir = (Q2Out-Q2L < 0)?backward:forward;
  Q3Dir = (Q3Out-Q3L < 0)?backward:forward;
  Q4Dir = (Q4Out-Q4L < 0)?backward:forward;

  //While the motors need to change (Q1L =! Q1) 
   bool Q1needchange=true;
   bool Q2needchange=true;
   bool Q3needchange=true;
   bool Q4needchange=true;    
      while(Q1needchange || Q2needchange || Q3needchange || Q4needchange){
          //If motor one still needs to change
          if(Q1L != Q1Out){
            tcaselect(Q1Port); //Open the port to the motor 1 DAC.
            dac.setVoltage(Q1L, true); //Set the voltage to send to the motor
            Q1L=scaledaccel(Q1L,Q1Out,Q1Dir); //Readjust change point        
          }
          else{Q1needchange=false;}
          
          //If motor two still needs to change
           if(Q2L != Q2Out){
            tcaselect(Q2Port); //Open the port to the motor 2 DAC.
            dac.setVoltage(Q2L, true); //Set the voltage to send to the motor
            Q1L=scaledaccel(Q2L,Q2Out,Q2Dir); //Readjust change point        
          }
          else{Q2needchange=false;}
          Serial.println(Q1L);
          //If motor three still needs to change
          if(Q3L != Q3Out){
            tcaselect(Q3Port); //Open the port to the motor 3 DAC.
            dac.setVoltage(Q3L, true); //Set the voltage to send to the motor
            Q1L=scaledaccel(Q3L,Q3Out,Q3Dir); //Readjust change point        
          }
          else{Q3needchange=false;}
          //If motor four still needs to change
          if(Q4L != Q4Out){
            tcaselect(Q4Port); //Open the port to the motor 4 DAC.
            dac.setVoltage(Q4L, true); //Set the voltage to send to the motor
            Q1L=scaledaccel(Q4L,Q4Out,Q4Dir); //Readjust change point        
          }
          else{Q4needchange=false;}
      delay(100); //I actually was not sure how much delay I should give to the incrementations.
      }
    }


int scaledaccel(int oval, int nval, bool isdecel){
//Auxillary function to accellimit
//Controls how quickly the DAC needs to change voltage depending on the difference in voltage represented using 12 bit resolution. 
int diff=0;
diff=abs(nval-oval);
if(isdecel){//If the WAMV is decelerating
    if(diff >= 1000){return oval-1000;}
    if(diff >= 100){return oval-100;}
    if(diff >= 10){return oval-10;}
    if(diff >= 1){return oval-1;}
}
else{ //If WAMV is accelerating
    if(diff >= 1000){return oval+1000;}
    if(diff >= 100){return oval+100;}
    if(diff >= 10){return oval+10;}
    if(diff >= 1){return oval+1;}
}
}




/*
 */
