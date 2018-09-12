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

void accellimits(int Q1L,int Q2L,int Q3L,int Q4L,int Q1Out,int Q2Out,int Q3Out,int Q4Out)
{
  //This program prevents sudden change in voltage to the motor controllers by slowly incrementing the voltage sent to 
  //the motor controllers to the desired voltage
  //Initiate I2c Communication with DACs
  dac.begin(0x62);
  const bool forward=true;
  const bool backward=false;
  const byte Q1Port=0;
  const byte Q2Port=1;
  const byte Q3Port=2;
  const byte Q4Port=3;
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
            dac.setVoltage(abs(Q1L), false); //Set the voltage to send to the motor
            Q1L=scaledaccel(Q1L,Q1Out,Q1Dir); //Readjust change point
            (Q1L<0)?qonereverse(hbridgepins,true):qonereverse(hbridgepins,false);        
          }
          else{Q1needchange=false;}
          
          //If motor two still needs to change
           if(Q2L != Q2Out){
            tcaselect(Q2Port); //Open the port to the motor 2 DAC.
            dac.setVoltage(abs(Q2L), false); //Set the voltage to send to the motor
            Q2L=scaledaccel(Q2L,Q2Out,Q2Dir); //Readjust change point
            (Q2L<0)?qtworeverse(hbridgepins,true):qtworeverse(hbridgepins,false);         
          }
          else{Q2needchange=false;}
          //If motor three still needs to change
          if(Q3L != Q3Out){
            tcaselect(Q3Port); //Open the port to the motor 3 DAC.
            dac.setVoltage(abs(Q3L), false); //Set the voltage to send to the motor
            Q3L=scaledaccel(Q3L,Q3Out,Q3Dir); //Readjust change point
            (Q3L<0)?qthreereverse(hbridgepins,true):qthreereverse(hbridgepins,false);         
          } 
          else{Q3needchange=false;}
          //If motor four still needs to change
          if(Q4L != Q4Out){
            tcaselect(Q4Port); //Open the port to the motor 4 DAC.
            dac.setVoltage(abs(Q4L), false); //Set the voltage to send to the motor
            Q4L=scaledaccel(Q4L,Q4Out,Q4Dir); //Readjust change point
            (Q4L<0)?qfourreverse(hbridgepins,true):qfourreverse(hbridgepins,false);         
          }
          else{Q4needchange=false;}
      delay(60); 
      }
    }



int scaledaccel(int oval, int nval, bool isaccel){
//Auxillary function to accellimit
//Controls how quickly the DAC needs to change voltage depending on the difference in voltage represented using 12 bit resolution. 
int diff=0;
diff=abs(nval-oval);
if(isaccel){//If the WAMV is decelerating
    if(diff >= 1000){return oval+200;}
    if(diff >= 100){return oval+50;}
    if(diff >= 10){return oval+5;}
    if(diff >= 1){return oval+1;}
}
else{ //If WAMV is accelerating
    if(diff >= 1000){return oval-200;}
    if(diff >= 100){return oval-50;}
    if(diff >= 10){return oval-5;}
    if(diff >= 1){return oval-1;}
}
}

void qonereverse(const byte hpins[16], bool reverse){
  //A function for controlling the H-bridge of motor controller 1 (Q1).
  if (reverse){ //If the motor does need to go in reverse
  digitalWrite(hpins[0],LOW); 
  digitalWrite(hpins[1],HIGH);
  digitalWrite(hpins[2],HIGH);
  digitalWrite(hpins[3],LOW);
  }
  else{ //If the motor is going forward
  digitalWrite(hpins[0],HIGH); 
  digitalWrite(hpins[1],LOW);
  digitalWrite(hpins[2],LOW);
  digitalWrite(hpins[3],HIGH);
  }
}

void qtworeverse(const byte hpins[16], bool reverse){
  //A function for controlling the H-bridge of motor controller 2 (Q2).
  if (reverse){ //If the motor does need to go in reverse
  digitalWrite(hpins[4],LOW); 
  digitalWrite(hpins[5],HIGH);
  digitalWrite(hpins[6],HIGH);
  digitalWrite(hpins[7],LOW);
  }
  else{ //If the motor is going forward
  digitalWrite(hpins[4],HIGH); 
  digitalWrite(hpins[5],LOW);
  digitalWrite(hpins[6],LOW);
  digitalWrite(hpins[7],HIGH);
  }
}

void qthreereverse(const byte hpins[16], bool reverse){
  //A function for controlling the H-bridge of motor controller 3 (Q3).
  if (reverse){ //If the motor does need to go in reverse
  digitalWrite(hpins[8],LOW); 
  digitalWrite(hpins[9],HIGH);
  digitalWrite(hpins[10],HIGH);
  digitalWrite(hpins[11],LOW);
  }
  else{ //If the motor is going forward
  digitalWrite(hpins[8],HIGH); 
  digitalWrite(hpins[9],LOW);
  digitalWrite(hpins[10],LOW);
  digitalWrite(hpins[11],HIGH);
  }
}

void qfourreverse(const byte hpins[16], bool reverse){
  //A function for controlling the H-bridge of motor controller 4 (Q4).
  if (reverse){ //If the motor does need to go in reverse
  digitalWrite(hpins[12],LOW); 
  digitalWrite(hpins[13],HIGH);
  digitalWrite(hpins[14],HIGH);
  digitalWrite(hpins[15],LOW);
  }
  else{ //If the motor is going forward
  digitalWrite(hpins[12],HIGH); 
  digitalWrite(hpins[13],LOW);
  digitalWrite(hpins[14],LOW);
  digitalWrite(hpins[15],HIGH);
  }
}


/*
 */
