#include <NewPing.h>   // include the NewPing library for this program
#include <Servo.h> //include the servo library for this program
#include <stdlib.h>

#define VCC_PIN 13
#define TRIGGER_PIN 12 // sonar trigger pin will be attached to Arduino pin 12
#define ECHO_PIN 11 // sonar echo pint will be attached to Arduino pin 11
#define GROUND_PIN 10
#define MAX_DISTANCE 200 // fmaximum distance set to 200 cm

#define GROUND_JOY_PIN A3 //joystick ground pin will connect to Arduino analog pin A3  
#define VOUT_JOY_PIN A2 //joystick +5 V pin will connect to Arduino analog pin A2  
#define XJOY_PIN A1 //  X axis reading from joystick will go into analog pin A1

#define SERVO_PIN 6

#define OPEN_POS 0
#define CLOSE_POS 180
#define GRAB_THRESHOLD 10
#define DROP_THRESHOLD 10

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // initialize NewPing

Servo servo; //create servo object to control a servo


void setup(){

  Serial. begin(9600);  // set data transmission rate to communicate with computer

  pinMode(ECHO_PIN, INPUT);  
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(GROUND_PIN, OUTPUT);  // tell pin 10 it is going to be an output
  pinMode(VCC_PIN, OUTPUT);  // tell pin 13 it is going to be an output

  digitalWrite(GROUND_PIN,LOW); // tell pin 10 to output LOW (OV, or ground)
  digitalWrite(VCC_PIN, HIGH) ; // tell pin 13 to output HIGH (+5V)
  pinMode(8,INPUT_PULLUP); //pin 8 forced to HIGH when there is no external input

  pinMode(VOUT_JOY_PIN, OUTPUT); //pin A3 shall be used as output
  pinMode(GROUND_JOY_PIN, OUTPUT) ; //pin A2 shall be used as output
  digitalWrite(VOUT_JOY_PIN, HIGH) ; //set pin A3 to high (+5)
  digitalWrite(GROUND_JOY_PIN,LOW) ; //set pin Ad to low (ground)
  
  servo.attach(SERVO_PIN); //attaches the servo on pin 6 to the servo object
}

//bool reset = false;
bool doneGrabbing = false;
bool stillGrabbing = true;
bool readyToDrop = false;
bool getBackUp = false;

void moveServo(int speed, int position);

//int grab(int dist, bool resetClaw);
//int drop(int dist, bool resetClaw);
void timedClaw(int dist);
void sensorClaw(int dist);

void loop(){
  
  delay(100); 
  
  int count = 0;
  for(count = 0; count < 5; count++){ //initialize sonar sensor
    int dist = sonar.ping_cm(); 
    delay(100);
  }

  int dist = sonar.ping_cm(); 
  Serial.print("Ping: "); //print “Ping:" on the computer display
  Serial.print(dist); //print the value of the variable next
  Serial.println("cm"); //print "cm" after that, then go to next line

  sensorClaw(dist);

//  if(DISTANCE_IN_CM < 6){
//    moveServo(5, 150, true);
//  }
//  else{
//    moveServo(5, 0, false);
//  }

//  if(stillGrabbing == true){
//    stillGrabbing = grab(dist, reset);
//  }
//  else{
//    stillGrabbing = drop(dist, reset);
//  }

//  if(reset == false){
//    grab(dist, reset);
//  }
//  if(reset == true){
//    drop(dist, reset);
//  }


}

void moveServo(int speed, int position){

  int currentPosition = servo.read();
  int pos;
  
  if(currentPosition < position){
    for (pos = currentPosition; pos <= position; pos += speed) {
      servo.write(pos);              
      delay(20);   
      Serial.println(servo.read());                    
    }
  }
  
  else{
    for (pos = currentPosition; pos >= position; pos -= speed) { 
      servo.write(pos);             
      delay(20);  
      Serial.println(servo.read());                     
    }
  }
}


void timedClaw(int dist){  
  if(dist < GRAB_THRESHOLD){
    delay(2000);    
    moveServo(5, OPEN_POS);
    delay(5000);
    moveServo(5, CLOSE_POS);
  }
}

void sensorClaw(int dist){
  
   if(dist < GRAB_THRESHOLD && readyToDrop == false && doneGrabbing == false && getBackUp == false){
    Serial.println("");
    Serial.println("Grabbing objects"); 
    delay(2000);    
    moveServo(5, CLOSE_POS);
    doneGrabbing = true; 
    Serial.println("Grabbed objects");     
   }
   if(doneGrabbing == true){
     if(dist > GRAB_THRESHOLD){
      readyToDrop = true;
     }
     if(readyToDrop == true && dist < DROP_THRESHOLD){
      Serial.println("");
      Serial.println("Dropping objects"); 
      moveServo(5, OPEN_POS);
      readyToDrop = false;
      doneGrabbing = false;
      getBackUp = true;
      Serial.println("Dropped objects"); 
     }
   }

   if(dist > DROP_THRESHOLD){
    getBackUp = false;
   }
}

/*
int grab(int dist, bool resetClaw){
  if(resetClaw == false  && dist < GRAB_THRESHOLD){
    if(doneGrabbing == false){
      delay(2000);
      Serial.println("got in the grab function");
      moveServo(5, CLOSE_POS);
      doneGrabbing == true;
    }
    return 1;
  }
  else{
    reset == true;
    return 0;
  } 
}

int drop(int dist, bool resetClaw){
  if(resetClaw == true  && dist < DROP_THRESHOLD){
    if(doneGrabbing == true){
      delay(2000);
      moveServo(5, OPEN_POS);
      doneGrabbing == false;
    }
    return 0;
  }
  else{
    reset == false;
    return 1;
  }
}
*/
