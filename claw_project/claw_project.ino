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

#define TRUE 1
#define FALSE 0

#define OPEN_POS 0
#define CLOSE_POS 150
#define GRAB_THRESHOLD 10
#define DROP_THRESHOLD 15

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

bool reset = FALSE;
bool doneGrabbing = FALSE;
bool stillGrabbing = TRUE;

void moveServo(int speed, int position, bool open);
int grab(int dist, bool resetClaw);
int release(int dist, bool resetClaw);

void loop(){

  delay(100); 
    
  int dist = sonar.ping_cm(); // read the sonar sensor, using a variable

  Serial.print("Ping: "); //print “Ping:" on the computer display
  Serial.print(dist); //print the value of the variable next
  Serial.println("cm"); //print "cm" after that, then go to next line

//  if(DISTANCE_IN_CM < 6){
//    moveServo(5, 150, TRUE);
//  }
//  else{
//    moveServo(5, 0, FALSE);
//  }

  if(stillGrabbing == TRUE){
    stillGrabbing = grab(dist, reset);
  }
  else{
    stillGrabbing = release(dist, reset);
  }
  


  //--------------------------- JOYSTICK --------------------------------------------------

//  delay(50);
//
//  int joystickXVal = analogRead(XJOY_PIN) ; //read joystick input on pin A1.  Will return a value between 0 and 1023.
//
//  int servoVal = map(joystickXVal, 0, 1023, 0, 180) ;  // changes the value to a raneg of 0 to 180.   See "map" function for further details.
//
//  Serial.print(joystickXVal); //print the value fram A1
//
//  Serial.println(" = input from joystick"); //print "=input from joystick" next to the value
//
//  Serial.print(servoVal); //print a from A1 calculated, scaled value
//
//  Serial.println(" = output to servo"); //print "=output to servo" next to the value
//
//  Serial.println() ;
//
//  servo.write(servoVal); //write the calculated value to the servo

}

void moveServo(int speed, int position, bool open){

  int currentPosition = servo.read();
  int pos;
  
  if(open == TRUE){
    for (pos = currentPosition; pos <= position; pos += speed) {
      servo.write(pos);              
      delay(20);                       
    }
  }
  
  else{
    for (pos = currentPosition; pos >= position; pos -= speed) { 
      servo.write(pos);             
      delay(20);                       
    }
  }
}

int grab(int dist, bool resetClaw){
  if(resetClaw == FALSE){
    if(doneGrabbing == FALSE && dist < GRAB_THRESHOLD){
      delay(2000);
      moveServo(5, CLOSE_POS, FALSE);
      doneGrabbing == TRUE;
    }
    return 1;
  }
  else{
    reset == TRUE;
    return 0;
  } 
}

int release(int dist, bool resetClaw){
  if(resetClaw == TRUE){
    if(doneGrabbing == TRUE && dist < DROP_THRESHOLD){
      delay(2000);
      moveServo(5, OPEN_POS, TRUE);
      doneGrabbing == FALSE;
    }
    return 0;
  }
  else{
    reset == FALSE;
    return 1;
  }
}
