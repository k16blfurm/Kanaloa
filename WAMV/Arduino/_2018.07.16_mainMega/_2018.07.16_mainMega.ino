// Library inclusions
#if (ARDUINO >= 100) //Check which version of Arduino we got
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h> // ROS functionalities
//#include <std_msgs/Bool.h> // Part of ROS
#include <Adafruit_PWMServoDriver.h> //For Servo Shield
#include<Wire.h> //To communicate with the servo shield: even if wire.h is not called explicitely, it is used by...
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver ();
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>

// Pin definitions
const int voltMainPin=A0;       // analog in from voltage divider pin for reading main battery voltage
const int tempPin=A3;           // analog in from temperature probe pin for reading temperature (optional)
const byte ch1Pin=25;     //Filler      // PWM in from remote control receiver channel ? (right stick left-to-right yaw/rotation) /*Need to figure out which stick motion on the controler coorelates to rotation*/
const byte ch3Pin=26;     //Filler      // PWM in from remote control receiver channel ? (left stick up-to-down surge/forward)    /*Need to figure out which stick motion on the controller coorelates to forward*/
const byte ch4Pin=5;            // PWM in from remote control receiver channel ? (left stick left-to-right sway/strafe)   /*Need to figure out which stick motion on the controller coorelates to strafe/pitch*/
const byte modePin= 7;          // PWM in from remote control receiver channel 4 (Switches between manual and autonomous modes)
const byte ch8Pin=11;     //Filler      // PWM in from remote control receiver channel 8 (remote kill switch)
const byte Q1Pin = 0;     //New      // PWM out to motor driver Q1 Note: These Q1-Q4 pins do not need to be set in the Arduino
const byte Q2Pin = 1;     //New      // PWM out to motor driver Q2
const byte Q3Pin = 2;     //New      // PWM out to motor driver Q3
const byte Q4Pin = 3;     //New      // PWM out to motor driver Q4
const byte yellowLightPin=37;   // digital out to yellow safety light relay (HIGH to turn on, LOW to turn off)
const byte greenLightPin=38;   // digital out to green safety light relay (HIGH to turn on, LOW to turn off)
const byte thrusterKillPin=16;  // digital out to thruster kill switch
const byte armReelKillPin=18;  // digital out to arm-reel kill switch



//States checker
int modeRead;

// Autonomous Command Signal from Matlab (via ROS)
int autoQ1;  
int autoQ2;
int autoQ3;
int autoQ4;
int invalid_input = 0;

/*Motor range values*/
const int fullreverse=194;
const int fullforward=392; //Old: 389
const int neutral=292;

/*PWM values*/
int manualPWM=1075; //When the manual takeover switch is down, it will have this value.
int controlmin;//Largest possible pulse width for control switch 
int controlmax;//smallest  possible pulse width for control switch
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

/*---------------------------------------------------------------------Initializing Servo Shield-----------------------------------------------------------------------------------------------*/
  //Sets PWM freq of adafruit servoshield to 50Hz. See data sheet http://www.revrobotics.com/content/docs/REV-11-1200-UM.pdf
  pwm.begin();
  pwm.setPWMFreq(50); 
/*---------------------------------------------------------------------Setting Pins------------------------------------------------------------------------------------------------------------*/
  // Set pins
  pinMode(ch3Pin, INPUT);
  pinMode(ch1Pin, INPUT);
  pinMode(ch4Pin, INPUT);
  pinMode(modePin, INPUT);
  pinMode(ch8Pin, INPUT);
  pinMode(yellowLightPin, OUTPUT); 
  pinMode(greenLightPin, OUTPUT); 
  pinMode(ch8Pin, INPUT); 
  pinMode(thrusterKillPin, OUTPUT);
  pinMode(armReelKillPin, OUTPUT);
  // Set serial baud rate 
  Serial.begin(9600);
  
}

void loop() {
  //Check state of kill switches
  killswitch();
  // Check mode (manual or auto) from transmitter
  modeRead = pulseIn(modePin,HIGH,35000);
  // ===================
  //     MANUAL CODE  
  // ====================

  if (modeRead<= manualPWM){   

    // Set safety light
    digitalWrite(yellowLightPin,HIGH);    // turn on yellow safety light
    digitalWrite(greenLightPin,LOW);      // turn off green safety light
    /*Add when transmitter works
     * 
     *  // Read in receiver values
    int ch1 = pulseIn(ch1Pin, HIGH, 32000);   // PWM in from remote control receiver channel 1 (right stick left-to-right yaw/rotation)
    int ch3 = pulseIn(ch3Pin, HIGH, 32000);   // PWM in from remote control receiver channel 3 (left stick up-to-down surge/forward)
    int ch4 = pulseIn(ch4Pin, HIGH, 32000);   // PWM in from remote control receiver channel 4 (left stick left-to-right sway/strafe)
 
    // Map receiver inputs from -100 to 100
    
    int controlmin= ;//Largest possible pulse width for control switch 
    int controlmax= ;//smallest  possible pulse width for control switch
    int controlmin;//Largest possible pulse width for control switch 
    int controlmax;//smallest  possible pulse width for control switch
    int ch1Map = map(ch1, controlmin, controlmax, -100, 100);   // yaw: 981 = full left. 1999 = full right. 
    int ch3Map = map(ch3, controlmin, controlmax, -100, 100);   // surge: 981 = full down. 1999 = full up.
    int ch4Map = map(ch4, controlmin, controlmax, 100, -100);   // sway: 978 = full left. 1996 = full right.

    // Calcuate surge, sway, and yaw components
    int surgeQ1 = ch3Map;
    int surgeQ2 = ch3Map;
    int surgeQ3 = ch3Map;
    int surgeQ4 = ch3Map;
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
     * 
     */
    
    
    
    
    int vel=-100;
    motormap(vel,vel,vel,vel,0);
   
  }

  // ====================
  //   AUTONOMOUS CODE
  // ====================

  else {
    //Just need to motormap autoQ's

    // Set safety light
    digitalWrite(yellowLightPin,LOW);     // turn off yellow safety light
    digitalWrite(greenLightPin,HIGH);     // turn on green safety light
    int vel = -100;
    motormap(vel,vel,vel,vel,0);
    // Spin once to refresh callbacks
    //nh.spinOnce();    // THIS ONE MIGHT NOT BE NEEDED
}

 // Small delay
  delay(10);

  // Spin once to refresh callbacks
  nh.spinOnce();

}







/*
Work on the function below as you get a better idea what needs to be done.
1.) Replace manchan with the global variable assosciated with that part.
2.) voltMain, may not be necessary
*/






void motormap(int Q1, int Q2, int Q3, int Q4, int voltMain){
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
    // Remaps surge, sway and yaw components to motor driver PWM duty cycle
    // From testing, at 50 Hz, full reverse = 194, and full forward = 389, neutral = 293 new
     Q1Out = map(Q1, -100, 100, fullreverse, fullforward);
     Q2Out = map(Q2, -100, 100, fullreverse, fullforward);
     Q3Out = map(Q3, -100, 100, fullreverse, fullforward);
     Q4Out = map(Q4, -100, 100, fullreverse, fullforward);
   }
   
   else{
    //If the reading has been already scaled because of autonomous mode
    Q1Out=Q1;
    Q2Out=Q2;
    Q3Out=Q3;
    Q4Out=Q4;
   }
    // Motor 
    // Adjust motor drive output based on main battery voltage
    int maxMotorVolt=12;
    //Q1Out = neutral+(Q1Out-neutral)*maxMotorVolt/voltMain;
    //Q2Out = neutral+(Q2Out-neutral)*maxMotorVolt/voltMain;
    //Q3Out = neutral+(Q3Out-neutral)*maxMotorVolt/voltMain;
    //Q4Out = neutral+(Q4Out-neutral)*maxMotorVolt/voltMain; 

    /*May need to adjust voltage formula above depending on what the 12 means...*/

      //If in manual mode...
      if (modeRead<=manualPWM){
      // Set a neutral "buffer" zone for manual transmitter
      if (Q1Out <= neutral+2 && Q1Out >=neutral-2) { Q1Out = neutral; }
      if (Q2Out <= neutral+2 && Q2Out >=neutral-2) { Q2Out = neutral; }
      if (Q3Out <= neutral+2 && Q3Out >=neutral-2) { Q3Out = neutral; }
      if (Q4Out <= neutral+2 && Q4Out >=neutral-2) { Q4Out = neutral; }
      }
Serial.println(modeRead);
    // Output to motor drivers
       pwm.setPWM(Q1Pin,0,Q1Out);
       pwm.setPWM(Q2Pin,0,Q2Out);
       pwm.setPWM(Q3Pin,0,Q3Out);
       pwm.setPWM(Q4Pin,0,Q4Out);


    // Publish to ROS
    Q1Msg.data = Q1Out;
    Q2Msg.data = Q2Out;
    Q3Msg.data = Q3Out;
    Q4Msg.data = Q4Out;
    Q1Pub.publish( &Q1Msg);
    Q2Pub.publish( &Q1Msg);
    Q3Pub.publish( &Q1Msg);
    Q4Pub.publish( &Q1Msg);
   
}

/*Note: We did not see nor test a pin for the kill switch. We may want to relook into that if things do not work out*/
void killswitch(){
  // Check kill states from remote kill switch
  int ch8 = pulseIn(ch8Pin, HIGH, 25000);

  // Kill as necessary, will need to change pulse width 
  if (ch8 >= 2000 && ch8 <= 2060) {
    digitalWrite(thrusterKillPin, HIGH);
    digitalWrite(armReelKillPin, HIGH); 
  }
  else if (ch8 >= 1485 && ch8 <= 1500){
    digitalWrite(thrusterKillPin, HIGH);
    digitalWrite(armReelKillPin, LOW); 
  }
  else {
    digitalWrite(thrusterKillPin, LOW);
    digitalWrite(armReelKillPin, LOW); 
  }
}

