/*
 * Dev notes: With this program, what we want it to do is, instead of using PWM, we need to send an Analog signal to the motor controller.
 */

//Change battery read function to read 36V (or maybe max 50V).
//Add manualPWM variable
//Make functions for each of the direction motor controller H-bridges






// Library inclusions
#if (ARDUINO >= 100) //Check which version of Arduino we got
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h> // ROS functionalities
//#include <std_msgs/Bool.h> // Part of ROS
#include <Adafruit_MCP4725.h> //For DACs
Adafruit_MCP4725 dac; //just a constructor
#include<Wire.h> //To communicate with the DACs
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>

//Definitions
#define TCAADDR 0x70

// Pin definitions
const int voltMainPin=A0;       // analog in from voltage divider pin for reading main battery voltage
const int tempPin=A3;           // analog in from temperature probe pin for reading temperature (optional)
const byte ch1Pin=4;            // PWM in from remote control receiver channel 1 (right stick left-to-right yaw/rotation) 
const byte ch6Pin=6;            // PWM in from remote control receiver channel 6 (left stick up-to-down surge/forward)   
const byte ch4Pin=5;            // PWM in from remote control receiver channel 4 (left stick left-to-right sway/strafe)   
const byte modePin= 7;          // PWM in from remote control receiver channel 5 (Switches between manual and autonomous modes)
const byte ch8Pin=11;           // PWM in from remote control receiver channel 8 (remote kill switch)
const byte yellowLightPin=37;   // digital out to yellow safety light relay (HIGH to turn on, LOW to turn off)
const byte greenLightPin=38;    // digital out to green safety light relay (HIGH to turn on, LOW to turn off)
const byte thrusterKillPin=16;  // digital out to thruster kill switch
const byte armReelKillPin=18;   // digital out to arm-reel kill switch
/*The pin values for yellowLight-armReal are made up*/
const byte hbridgepins[16]={16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31}; 
//Every 4 values of array corresponds to 1 set of H-bridge connections to motor controllers (first 4 go to Q1, last 4 go to Q4), used for H-bridge functions
//For a set of 4 H-bridge pins, the order of the values matter. Here is how each pin value corresponds to a set of 4 values. 
//First  value of 4: pforward, green P+ 
//Second value of 4: pbackwards, yellow P- 
//Third  value of 4: nforward, red N+ 
//Fourth value of 4: nbackwards, blue N- 

//Note: Pin 3 and Pin 2 on main arduino is not working. 

//States checker
int modeRead; //Checks whether the boat is in automatic or manual mode
int voltMain; //Checks the voltage from the main battery


//Transmitter values
int controlmin=1075;//Largest possible pulse width for control switchs (the joysticks)
int controlmax=1900;//smallest  possible pulse width for control switchs (the joysticks)
int manualPWM=1100; //Checks whether the boat is in manual or autonamous mode. When the manual takeover switch is down, it will have this value.

// Autonomous Command Signal from Matlab (via ROS)
int autoQ1;  
int autoQ2;
int autoQ3;
int autoQ4;
int invalid_input = 0;

/*Motor range values*/
const int fullreverse= -2948; //194 Was the old All but 1 blink. 192 all but 1 solid. Basically, if we want to be really precise, we would need to find each individual reverse and forward.
const int fullforward=2948; //This value causes one of the motors to be solid 
const int neutral=0;

/*-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

//ROS Management

// ROS node handle (allows program to create publishers and subscribers)
ros::NodeHandle nh;

// ROS messages
std_msgs::UInt16 Q1Msg;         // Q1 thruster message
std_msgs::UInt16 Q2Msg;         // Q2 thruster message
std_msgs::UInt16 Q3Msg;         // Q3 thruster message 
std_msgs::UInt16 Q4Msg;         // Q4 thruster message
std_msgs::Float64 voltMainMsg;  // main battery voltage message
 
// ROS publishers
ros::Publisher voltMainPub("voltMain",&voltMainMsg);  // main battery voltage
ros::Publisher Q1Pub("Q1", &Q1Msg);                   // Q1 thruster output  
ros::Publisher Q2Pub("Q2", &Q2Msg);                   // Q2 thruster output
ros::Publisher Q3Pub("Q3", &Q3Msg);                   // Q3 thruster output
ros::Publisher Q4Pub("Q4", &Q4Msg);                   // Q4 thruster output

// ROS callback functions (must come before subscibers)
void autoQ1Cb(const std_msgs::Float64& autoQ1CbMsg) {
  autoQ1 = map(autoQ1CbMsg.data, -100, 100, fullreverse, fullforward);
}
void autoQ2Cb(const std_msgs::Float64& autoQ2CbMsg) {
  autoQ2 = map(autoQ2CbMsg.data, -100, 100, fullreverse, fullforward);
}
void autoQ3Cb(const std_msgs::Float64& autoQ3CbMsg) {
  autoQ3 = map(autoQ3CbMsg.data, -100, 100, fullreverse, fullforward);
}
void autoQ4Cb(const std_msgs::Float64& autoQ4CbMsg) {
  autoQ4 = map(autoQ4CbMsg.data, -100, 100, fullreverse, fullforward);
}

// ROS Subscribers
ros::Subscriber<std_msgs::Float64> autoQ1sub("autoQ1", &autoQ1Cb);   // autonomous Q1 thruster output
ros::Subscriber<std_msgs::Float64> autoQ2sub("autoQ2", &autoQ2Cb);   // autonomous Q2 thruster output 
ros::Subscriber<std_msgs::Float64> autoQ3sub("autoQ3", &autoQ3Cb);   // autonomous Q3 thruster output
ros::Subscriber<std_msgs::Float64> autoQ4sub("autoQ4", &autoQ4Cb);   // autonomous Q4 thruster output


void setup() {

  // ROS Node handle
  nh.initNode();

  // Setup publishers
  nh.advertise(voltMainPub);    // main battery voltage
  nh.advertise(Q1Pub);          // Q1 thruster output
  nh.advertise(Q2Pub);          // Q2 thruster output
  nh.advertise(Q3Pub);          // Q3 thruster output
  nh.advertise(Q4Pub);          // Q4 thruster output

  // Setup subscibers
  nh.subscribe(autoQ1sub);    // autonomous Q1 thruster output
  nh.subscribe(autoQ2sub);    // autonomous Q2 thruster output
  nh.subscribe(autoQ3sub);    // autonomous Q3 thruster output
  nh.subscribe(autoQ4sub);    // autonomous Q4 thruster output

/*---------------------------------------------------------------------Initializing DACs-----------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------Setting Pins------------------------------------------------------------------------------------------------------------*/
  // Set pins
  pinMode(ch6Pin, INPUT);
  pinMode(ch1Pin, INPUT);
  pinMode(ch4Pin, INPUT);
  pinMode(modePin, INPUT);
  pinMode(ch8Pin, INPUT);
  pinMode(yellowLightPin, OUTPUT); 
  pinMode(greenLightPin, OUTPUT); 
  pinMode(ch8Pin, INPUT); 
  pinMode(thrusterKillPin, OUTPUT);
  pinMode(armReelKillPin, OUTPUT);

  for(int pin=0;pin<sizeof(hbridgepins)/sizeof(hbridgepins[0]); pin++){
    pinMode(hbridgepins[pin],OUTPUT); 
    } //Shortcut for setting all the pins for the H-bridges

    //Initialize DACs
    dac.begin(0x62);

  // Set serial baud rate 
  //Serial.begin(9600); //If connected to a laptop.
  Serial.begin(57600); //If connected to onboard computer
}

//For Acceleration Limits
//initialization
int Q1Last=0;
int Q2Last=0;
int Q3Last=0;
int Q4Last=0;

void loop() {
  //Take a reading of the main battery
    voltMain=batteryRead();
  // Check mode (manual or auto) from transmitter
    modeRead= pulseIn(modePin,HIGH); //1081 pulse width for low;  //pulseIn(modePin,HIGH);
  // ====================
  //     MANUAL CODE  
  // ====================

  if (modeRead<=manualPWM) {  // if WAMV is in manual mode

    // Set safety light
    digitalWrite(yellowLightPin,HIGH);    // turn on yellow safety light
    digitalWrite(greenLightPin,LOW);      // turn off green safety light
   
    //Read signals from transmitter and encode it for motors
    unsigned long ch1 = pulseIn(ch1Pin, HIGH);   // PWM in from remote control receiver channel 1 (right stick left-to-right yaw/rotation) Full Left: 1075 Full Right: 1900
    unsigned long ch6 = pulseIn(ch6Pin, HIGH);   // PWM in from remote control receiver channel 6 (left stick up-to-down surge/forward) Same range
    unsigned long ch4 = pulseIn(ch4Pin, HIGH);   // PWM in from remote control receiver channel 4 (left stick left-to-right sway/strafe) Full Left: 1075 Full Right: 1900

    // Map receiver inputs from -100 to 100
    int ch1Map = map(ch1, controlmin, controlmax, -100, 100);   // yaw: 1080 = full left. 1891 = full right. 
    int ch6Map = map(ch6, controlmin, controlmax, -100, 100);   // surge: 1080 = full down. 1891 = full up.
    int ch4Map = map(ch4, controlmin, controlmax, 100, -100);   // sway: 1080 = full left. 1891 = full right.  

    int surgeQ1 = ch6Map;
    int surgeQ2 = ch6Map;
    int surgeQ3 = ch6Map;
    int surgeQ4 = ch6Map;
    int swayQ1 = ch4Map;
    int swayQ2 = -ch4Map;
    int swayQ3 = ch4Map;
    int swayQ4 = -ch4Map;
    int yawQ1 = -ch1Map;
    int yawQ2 = ch1Map;
    int yawQ3 = ch1Map;
    int yawQ4 = -ch1Map;

    // Sum surge, sway, and yaw components
    int Q1Map = surgeQ1+swayQ1+yawQ1;
    int Q2Map = surgeQ2+swayQ2+yawQ2;
    int Q3Map = surgeQ3+swayQ3+yawQ3;
    int Q4Map = surgeQ4+swayQ4+yawQ4;

    // Limit components to no less than -100 and no more than 100
    constrain(Q1Map,-100,100);
    constrain(Q2Map,-100,100);
    constrain(Q3Map,-100,100);
    constrain(Q4Map,-100,100);
    
    //Adjust and send values to motors
    motormap(Q1Map,Q2Map,Q3Map,Q4Map,Q1Last,Q2Last,Q3Last,Q4Last);
    //Save used values for acceleration limits
    Q1Last=Q1Map;
    Q2Last=Q2Map;
    Q3Last=Q3Map;
    Q4Last=Q4Map;
  }

  // ====================
  //   AUTONOMOUS CODE
  // ====================

  else {
    // Set safety light
    digitalWrite(yellowLightPin,LOW);     // turn off yellow safety light
    digitalWrite(greenLightPin,HIGH);     // turn on green safety light
    //Send signals from autonomous source to motors.
    motormap(autoQ1,autoQ2,autoQ3,autoQ4,Q1Last,Q2Last,Q3Last,Q4Last);
    //Store previous values
    Q1Last=autoQ1;
    Q2Last=autoQ2;
    Q3Last=autoQ3;
    Q4Last=autoQ4;
}
 // Small delay
  delay(10);

  // Spin once to refresh callbacks
  nh.spinOnce();
}

float batteryRead(){
  // Main battery voltage reader
  float voltMainBit = analogRead(voltMainPin);      // voltage on voltMainPin [bit]
  float mainVolt = voltMainBit *(25.0/1023.0);         // voltage on voltMainPin [V] Was 21.0/1024.0 /*This will need to be changed. This thing will be reading 50V, currently it is scaled to read 15V).
  voltMainMsg.data = mainVolt;                      // set voltMain message
  voltMainPub.publish( &voltMainMsg);               // publish voltMain topic
  return mainVolt;
}


void motormap(int Q1, int Q2, int Q3, int Q4, int Q1Last, int Q2Last, int Q3Last, int Q4Last){
  /*
   * This function takes in values of movement for each motor, remaps it to the motor's PWM duty cycle, 
   * and sends the result after adjustment to the motors and through ROS.
   */
   int Q1Out;
   int Q2Out;
   int Q3Out;
   int Q4Out;
   if (modeRead<=manualPWM){
    // If we are in manual mode
    // Remaps surge, sway and yaw components to motor driver analog signal range based on 12 bit resolution
    // From testing, at 50 Hz, full reverse = -2948, and full forward = 2948, neutral = 0
     Q1Out = map(Q1, -100, 100, fullreverse, fullforward);
     Q2Out = map(Q2, -100, 100, fullreverse, fullforward);
     Q3Out = map(Q3, -100, 100, fullreverse, fullforward);
     Q4Out = map(Q4, -100, 100, fullreverse, fullforward);
   }
    //Proportions: We can use 36/50 V, in bits thats 2948/4095 
   
   else{
    //If the reading has already been encoded because of autonomous mode
    Q1Out=Q1;
    Q2Out=Q2;
    Q3Out=Q3;
    Q4Out=Q4;
   }
   
    // Adjust motor drive output based on main battery voltage
    const byte maxThrusterVolt=12;
    Q1Out = neutral+(Q1Out-neutral)* maxThrusterVolt/voltMain;
    Q2Out = neutral+(Q2Out-neutral)* maxThrusterVolt/voltMain;
    Q3Out = neutral+(Q3Out-neutral)* maxThrusterVolt/voltMain;
    Q4Out = neutral+(Q4Out-neutral)* maxThrusterVolt/voltMain; 

    /*Buffer zone*/
      //If in manual mode...
      if (modeRead<=manualPWM){
      // Set a neutral "buffer" zone for manual transmitter
      if (Q1Out <= neutral+2 && Q1Out >=neutral-2) { Q1Out = neutral; }
      if (Q2Out <= neutral+2 && Q2Out >=neutral-2) { Q2Out = neutral; }
      if (Q3Out <= neutral+2 && Q3Out >=neutral-2) { Q3Out = neutral; }
      if (Q4Out <= neutral+2 && Q4Out >=neutral-2) { Q4Out = neutral; }
      }
      
    // Output to motor drivers
     //accellimits(Q1Last,Q2Last,Q3Last,Q4Last,Q1Out,Q2Out,Q3Out,Q4Out);

    
    // Publish to ROS (For troubleshooting);
    Q1Msg.data = Q1Out;
    Q2Msg.data = Q2Out;
    Q3Msg.data = Q3Out;
    Q4Msg.data = Q4Out;
    Q1Pub.publish( &Q1Msg);
    Q2Pub.publish( &Q2Msg);
    Q3Pub.publish( &Q3Msg);
    Q4Pub.publish( &Q4Msg);
}


void tcaselect(uint8_t i) {
  //Opens Ports on the multiplexer to communicate with individual DACs
  //i is the number of the port
  //Provided by Adafruit Documentation
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

//Add acceleration limits code when finished from accellimits file



void qonereverse(const byte hpins[16], bool reverse){
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
