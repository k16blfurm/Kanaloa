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
     //Ports on Multiplexers
     const byte Q1Port=0;
     const byte Q2Port=1;
     const byte Q3Port=3;
     const byte Q4Port=2;
  //Controlling Acceleration    
      while(Q1L<Q1Out || Q2L< Q2Out || Q3L<Q3Out || Q4L<Q4Out){
          //Output Voltage to Q1
          bool isdecel=false;
          tcaselect(Q1Port);
          dac.setVoltage(Q1L, true); 
            if (Q1L<Q1Out){Q1L=scaledaccel(Q1L,Q1Out,isdecel);} //If the old value is not the same as the new value, increment closer the new voltage
          //Output Voltage to Q2
          tcaselect(Q2Port);
          dac.setVoltage(Q2L, true);
            if (Q2L<Q2Out){Q2L=scaledaccel(Q2L,Q2Out,isdecel);}
          //Output to Q3
          tcaselect(Q3Port);
          dac.setVoltage(Q3L, true);
            if(Q3L<Q3Out){Q3L=scaledaccel(Q3L,Q3Out,isdecel);}
          //Output to Q4
          tcaselect(Q4Port);
          dac.setVoltage(Q4Out, true);
            if (Q4L<Q4Out){Q4L=scaledaccel(Q4L,Q4Out,isdecel);}
      delay(100); //I actually was not sure how much delay I should give to the incrementations.
      }
      //Controlling Deceleration 
    while(Q1L>Q1Out || Q2L > Q2Out || Q3L >Q3Out || Q4L >Q4Out){
          //Output to Q1
          bool isdecel=true;
          tcaselect(Q1Port);
          dac.setVoltage(Q1L, true); 
            if (Q1L>Q1Out){Q1L=scaledaccel(Q1L,Q1Out,isdecel);} //Slowly Speed up (increment by 1 bit)
          //Output to Q2
          tcaselect(Q2Port);
          dac.setVoltage(Q2L, true);
            if (Q2L>Q2Out){Q2L=scaledaccel(Q2L,Q2Out,isdecel);}
          //Output to Q3
          tcaselect(Q3Port);
          dac.setVoltage(Q3L, true);
            if(Q3L>Q3Out){Q3L=scaledaccel(Q3L,Q3Out,isdecel);}
          //Output to Q4
          tcaselect(Q4Port);
          dac.setVoltage(Q4Out, true);
            if (Q4L>Q4Out){Q4L=scaledaccel(Q4L,Q4Out,isdecel);}
      delay(100);
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
