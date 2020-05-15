# APSC-Claw-Project
Claw project code for APSC 101 

 ### Table of Contents
 * [Move Servo](#Move-Servo)
 * [Sensor Based Control](#Sensor-Based-Control)
 * [Weighted Average Calculation](#Weighted-Average-Calculation)

### Move Servo

The default **move** function moves the servo at the maximum speed possible which was caused problems while picking up objects. Therefore, this **moveServo** function allows for servo speed control. 

```cpp
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
```

### Sensor Based Control

Using the **sensorClaw** function, the claw automatically opens at closes at preset heights by using distance data collected by the sonar sensor. This fucntion uses flags to change states between the *dropping* and the *grabbing* phases.

```cpp
void sensorClaw(double dist){
  
   if(dist < GRAB_THRESHOLD && readyToDrop == false && doneGrabbing == false && getBackUp == false){
    Serial.println("");
    Serial.println("Grabbing objects"); 
    delay(1000);    
    moveServo(4, CLOSE_POS);
    doneGrabbing = true; 
    Serial.println("Grabbed objects");
    digitalWrite(LED, HIGH);     
    failsafe = millis();
   }
   if(doneGrabbing == true){    
     if(dist > GRAB_THRESHOLD){
      readyToDrop = true;      
     }
     if(readyToDrop == true && dist < DROP_THRESHOLD || failsafe + FAILSAFE < millis()){
      Serial.println("");
      Serial.println("Dropping objects"); 
      moveServo(10, OPEN_POS);
      readyToDrop = false;
      doneGrabbing = false;
      getBackUp = true;
      Serial.println("Dropped objects"); 
      digitalWrite(LED, LOW); 
     }
   }

   if(dist > DROP_THRESHOLD){
    getBackUp = false;
   }
}
```

### Weighted Average Calculation

The **weightedAvg** function is the most important since it eliminates any inconsistencies in the sonar sensor. Since the sonar is easily disturbed, it may give faulty value randomly which are far off compared to an older sensor value. This function weighs the new readings as lower than previous distance readings which ignores the effects of random values. The function settles to a steady-state value in about 200-300ms.

```cpp
double weightedAvg(double dist){
  if(dist != 0){
   prevDist = prevDist * 0.9 + dist * 0.1;
  }
  return prevDist; 
}
```
