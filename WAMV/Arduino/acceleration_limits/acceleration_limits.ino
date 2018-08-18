void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  /*
  for(int i=0,j=0,k=0,m=0; i<=4 || j<=7 || k<=20 || m<=30; i++,j++,k++,m++)
  {
    //Steady the voltages for each of the diff
    if (i>4){i--;}
    if (j>7){j--;}
    if (k>20){k--;}
    if (m>30){m--;}
    Serial.print("i = ");
    Serial.println(i);
    Serial.print("j= ");
    Serial.println(j);
    Serial.print("k= ");
    Serial.println(k);
    Serial.print("m= ");
    Serial.println(m);
  }
  */
  accellimit(1024,2048,3072,4096, -1024, -2048, -3072, -4096);
}

void accellimit(int Q1L,int Q2L,int Q3L,int Q4L,int Q1Out,int Q2Out,int Q3Out,int Q4Out)
{
  //This program prevents sudden change in voltage to the motor controllers by slowly incrementing the voltage sent to 
  //the motor controllers to the desired voltage
  //Initiate I2c Communication with DACs
  //If you are accelerating too fast    
      while(Q1L<Q1Out || Q2L< Q2Out || Q3L<Q3Out || Q4L<Q4Out){
          //Output to Q1
          bool isdecel=false; 
         Serial.print("Q1L= ");
         Serial.println(Q1L); 
            if (Q1L<Q1Out){Q1L=scaledaccel(Q1L,Q1Out,isdecel);}
          //Output to Q2
         Serial.print("Q2L= ");
         Serial.println(Q2L); 
            if (Q2L<Q2Out){Q2L=scaledaccel(Q2L,Q2Out,isdecel);}
          //Output to Q3
         Serial.print("Q3L= ");
         Serial.println(Q3L); 
            if(Q3L<Q3Out){Q3L=scaledaccel(Q3L,Q3Out,isdecel);}
          //Output to Q4
         Serial.print("Q4L= ");
         Serial.println(Q4L); 
            if (Q4L<Q4Out){Q4L=scaledaccel(Q4L,Q4Out,isdecel);}
      }
      //Decelerating too quickly
    while(Q1L>Q1Out || Q2L> Q2Out || Q3L>Q3Out || Q4L>Q4Out){
         bool isdecel=true;
          //Output to Q1
         Serial.print("Q1L= ");
         Serial.println(Q1L); 
            if (Q1L>Q1Out){Q1L=scaledaccel(Q1L,Q1Out,isdecel);} //Slowly Speed up (increment by 1 bit)
          //Output to Q2
         Serial.print("Q2L= ");
         Serial.println(Q2L); 
            if (Q2L>Q2Out){Q2L=scaledaccel(Q2L,Q2Out,isdecel);}
          //Output to Q3
         Serial.print("Q3L= ");
         Serial.println(Q3L); 
            if(Q3L>Q3Out){Q3L=scaledaccel(Q3L,Q3Out,isdecel);}
          //Output to Q4
         Serial.print("Q4L= ");
         Serial.println(Q4L); 
            if (Q4L>Q4Out){Q4L=scaledaccel(Q4L,Q4Out,isdecel);}
         delay(1000);
      }

}

int scaledaccel(int oval, int nval, bool isdecel){
//Auxillary function to accellimit
//Controls how quickly the DAC needs to change voltage depending on the difference in voltage. 
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



/*COPY THESE LINES
 * void accellimit(int Q1L,int Q2L,int Q3L,int Q4L,int Q1Out,int Q2Out,int Q3Out,int Q4Out)
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
          //Output to Q1
          bool isdecel=false;
          tcaselect(Q1Port);
          dac.setVoltage(Q1L, true); 
            if (Q1L<Q1Out){Q1L=scaledaccel(Q1L,Q1Out,isdecel);} //Slowly Speed up (increment by 1 bit)
          //Output to Q2
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
      delay(100);
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
//Controls how quickly the DAC needs to change voltage depending on the difference in voltage. 
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
 */




