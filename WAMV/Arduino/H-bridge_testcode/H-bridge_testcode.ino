byte pforward=6; //green P+ (not)
byte nforward=7;// red N+ (not)
byte pbackward=12; //yellow P- (working)
byte nbackward=11; //blue N- (was working)

void setup() {
  pinMode(pforward,OUTPUT);
  pinMode(nforward,OUTPUT);
  pinMode(pbackward,OUTPUT);
  pinMode(nbackward,OUTPUT);
}

void loop(){
  //put your main code here, to run repeatedly:
  //The commented out code is testing for forward, for a second and backward for 2 seconds.
  /*
  digitalWrite(pforward,HIGH); 
  digitalWrite(pbackward,LOW);
  digitalWrite(nforward,LOW);
  digitalWrite(nbackward,HIGH);
  delay(1000);
  digitalWrite(pforward,LOW); 
  digitalWrite(pbackward,HIGH);
  digitalWrite(nforward,HIGH);
  digitalWrite(nbackward,LOW);
  delay(2000);
  */

  //This code is testing to see if we fried anything see lines 1-4 above for continuity test results...
  digitalWrite(pforward,HIGH);
  delay(1000);
  digitalWrite(pforward,LOW);
  delay(1000);
  //reset();
}

void reset(void){
digitalWrite(pforward,LOW);
digitalWrite(nforward,LOW);
digitalWrite(pbackward,LOW);
digitalWrite(nbackward,LOW);
}
