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
const byte ch1Pin=4;            // PWM in from remote control receiver channel 1 (right stick left-to-right yaw/rotation) 
const byte ch6Pin=6;            // PWM in from remote control receiver channel 6 (left stick up-to-down surge/forward)   
const byte ch4Pin=5;            // PWM in from remote control receiver channel 4 (left stick left-to-right sway/strafe)   
const byte modePin= 7;          // PWM in from remote control receiver channel 5 (Switches between manual and autonomous modes)
const byte ch8Pin=11;           // PWM in from remote control receiver channel 8 (remote kill switch)
const byte Q1Pin = 0;     //New      // PWM out to motor driver Q1
const byte Q2Pin = 1;     //New      // PWM out to motor driver Q2
const byte Q3Pin = 2;     //New      // PWM out to motor driver Q3
const byte Q4Pin = 3;     //New      // PWM out to motor driver Q4
const byte yellowLightPin=37;   // digital out to yellow safety light relay (HIGH to turn on, LOW to turn off)
const byte greenLightPin=38;    // digital out to green safety light relay (HIGH to turn on, LOW to turn off)
const byte thrusterKillPin=16;  // digital out to thruster kill switch
const byte armReelKillPin=18;   // digital out to arm-reel kill switch
/*The pin values for yellowLight-armReal are made up*/


//Note: Pin 3 and Pin 2 on main arduino is not working. 

//States checker
int modeRead; //Checks whether the boat is in automatic or manual mode
int voltMain; //Checks the voltage from the main battery

// Autonomous Command Signal from Matlab (via ROS)
int autoQ1;  
int autoQ2;
int autoQ3;
int autoQ4;
int invalid_input = 0;

/*Motor range values*/
const int fullreverse=193; //194 Was the old All but 1 blink. 192 all but 1 solid. Basically, if we want to be really precise, we would need to find each individual reverse and forward.
const int fullforward=389; //This value causes one of the motors to be solid 
const int neutral=292;

/*PWM values*/
int manualPWM=1100; //When the manual takeover switch is down, it will have this value.
int controlmin=1075;//Largest possible pulse width for control switch 
int controlmax=1900;//smallest  possible pulse width for control switch
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
  pinMode(ch6Pin, INPUT);
  pinMode(ch1Pin, INPUT);
  pinMode(ch4Pin, INPUT);
  pinMode(modePin, INPUT);
  pinMode(ch8Pin, INPUT);
  pinMode(Q1Pin, OUTPUT);
  pinMode(Q2Pin, OUTPUT); 
  pinMode(Q3Pin, OUTPUT);
  pinMode(Q4Pin, OUTPUT);
  pinMode(yellowLightPin, OUTPUT); 
  pinMode(greenLightPin, OUTPUT); 
  pinMode(ch8Pin, INPUT); 
  pinMode(thrusterKillPin, OUTPUT);
  pinMode(armReelKillPin, OUTPUT);

  // Set serial baud rate 
  Serial.begin(9600);
}

void loop() {
  //Check if any of the kill switches needs to be turned on or off.
  //killswitch();
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
    //transmittermap(*Q1,*Q2, *Q3, *Q4); //This function does not work
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
    Serial.println(ch6Map);
    
    //Adjust and send values to motors
    motormap(Q1Map,Q2Map,Q3Map,Q4Map);
  }

  // ====================
  //   AUTONOMOUS CODE
  // ====================

  else {
    // Set safety light
    digitalWrite(yellowLightPin,LOW);     // turn off yellow safety light
    digitalWrite(greenLightPin,HIGH);     // turn on green safety light
    //Send signals from autonomous source to motors.
    motormap(autoQ1,autoQ2,autoQ3,autoQ4);
}
 // Small delay
  delay(10);

  // Spin once to refresh callbacks
  nh.spinOnce();
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

float batteryRead(){
  // Main battery voltage reader
  float voltMainBit = analogRead(voltMainPin);      // voltage on voltMainPin [bit]
  float mainVolt = voltMainBit *(25.0/1023.0);         // voltage on voltMainPin [V] Was 21.0/1024.0
  voltMainMsg.data = mainVolt;                      // set voltMain message
  voltMainPub.publish( &voltMainMsg);               // publish voltMain topic
  return mainVolt;
}


void motormap(int Q1, int Q2, int Q3, int Q4){
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
    // (31 = full reverse, 47 = neutral, 63 = full forward) Old
    // From testing, at 50 Hz, full reverse = 194, and full forward = 389, neutral = 293 new
     Q1Out = map(Q1, -100, 100, fullreverse, fullforward);
     Q2Out = map(Q2, -100, 100, fullreverse, fullforward);
     Q3Out = map(Q3, -100, 100, fullreverse, fullforward);
     Q4Out = map(Q4, -100, 100, fullreverse, fullforward);
   }
   
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
    Serial.println(Q1Out);
    //Serial.println(voltMain);
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


/*Scrapped but explore further if possible
 * 
 * The below is a function for mapping transmission signals. There is an error that causes the Arduino to not properly read values from pulseIn.
  */
   /*
    //If you do not know how pointers work, I apologize for this system. Update: This was part of a function. That function does not work.
    int Q1Map=0;
    int Q2Map=0;
    int Q3Map=0;
    int Q4Map=0;
    int *Q1;
    int *Q2;
    int *Q3;
    int *Q4;
    //Basically, with pointers, the values that go to Q1 to Q4 will be assigned to Q1Map to Q4Map respectively.
    *Q1 = &Q1Map;
    *Q2 = &Q2Map;
    *Q3 = &Q3Map;
    *Q4 = &Q4Map;
    *
     */
 /* void transmittermap(int *Q1,int *Q2,int *Q3,int *Q4){
  //Adjust PWM values from the transmitter for the motor 
  
  // Read in receiver values
    unsigned long ch1 = pulseIn(ch1Pin, HIGH);   // PWM in from remote control receiver channel 1 (right stick left-to-right yaw/rotation) Full Left: 1075 Full Right: 1900
    unsigned long ch6 = pulseIn(ch6Pin, HIGH);   // PWM in from remote control receiver channel 6 (left stick up-to-down surge/forward) Same range
    unsigned long ch4 = pulseIn(ch4Pin, HIGH);   // PWM in from remote control receiver channel 4 (left stick left-to-right sway/strafe) Full Left: 1075 Full Right: 1900

    // Map receiver inputs from -100 to 100
    int ch1Map = map(ch1, controlmin, controlmax, -100, 100);   // yaw: 1080 = full left. 1891 = full right. 
    int ch6Map = map(ch6, controlmin, controlmax, -100, 100);   // surge: 1080 = full down. 1891 = full up.
    int ch4Map = map(ch4, controlmin, controlmax, 100, -100);   // sway: 1080 = full left. 1891 = full right.    

    // Calcuate surge, sway, and yaw components
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
    //Return adjusted signals
    *Q1=Q1Map;
    *Q2=Q2Map;
    *Q3=Q3Map;
    *Q4=Q4Map;
}
 */
