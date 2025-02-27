#include <Motor_TB.h>

/*
threshold goal 985-989 / 1000

670-720 - outside the square

robot_fence_lib_v01a_copy_20250123195454

photovore_lib_test_v06

ldr_readout_test_v01g_copy_20250124215742

robot_buddy_v03

A red LED is ON when the Robot is driving forward and flashes when the robot is turning.

A yellow LED is ON when Robot is stopped and flashes when the robot backs up.

A green LED is ON and solid when the robot is on but isn't moving.

A blue LED flashes as it gets closer to the beacon (closer = faster flashes, farther = slower flashes): increment var++(*++var)/var-- (possible +=var)

We use our custom sensor array to detect the beacon and drive the robot autonomously towards the beacon.
*/


Motor_TB robot(9,8,7,3,5,4,11,10,2,13);

//global vars
//IR sensors
//Dark = 1 | White = 0
int leftIR = 6;
int rightIR = 12;

int  leftIRdata;
int  rightIRdata;

//LDRs
int Left = A1;
int Center = A2;
int Right = A3;

int data[3];
//int ldrThreshold = 750;
int ldrThreshold = 450;
int ldrGoal = 980;



void setup() {
  // put your setup code here, to run once:
  //LDR pins
  pinMode(Left, INPUT);
  pinMode(Center, INPUT);
  pinMode(Right, INPUT);
 
  Serial.begin(9600);
  Serial.println("Intialization Complete!");

  robot.go(100,-100);
  delay(750);
  robot.stopMotors();
}

void getReadings(){
  data[0] = analogRead(Left);
  data[1] = analogRead(Center);
  data[2] = analogRead(Right);

  Serial.print("Left = ");
  Serial.print(data[0]);
  Serial.print(" | Center = ");
  Serial.print(data[1]);
  Serial.print(" | Right = ");
  Serial.println(data[2]);
}

int getLargest(){
  data[0] = analogRead(Left);
  data[1] = analogRead(Center);
  data[2] = analogRead(Right);


  if(data[0] < ldrThreshold && data[1] < ldrThreshold && data[2] < ldrThreshold){
    Serial.println("stopping...looking for light source");
    return -1;
  }
  else if(data[1] > data[0] && data[1] > data[2]){
    Serial.print("Highest reading - ");  
    Serial.print(data[1]);  
    Serial.println(" | going forward");
    return data[1];
  }
  else if(data[0] > data[1] && data[0] > data[2]){
    Serial.print("Highest reading - ");  
    Serial.print(data[0]);  
    Serial.println(" | turing left");
    return data[0];
  }
  else if(data[2] > data[1] && data[2] > data[0]){
    Serial.print("Highest reading - ");  
    Serial.print(data[2]);  
    Serial.println(" | turing right"); 
    return data[2];
  }
}
/*
void driveToBrightness(){
  int highVal = getLargest();
  if(highVal == -1){
    robot.go(0,0);
  }
  else if (highVal == data[1]){
    robot.go(150,100);
  }
  else if(highVal == data[0]){
    robot.go(-150,100);
  }
  else if(highVal == data[2]){
    robot.go(150,-100);
  }
  else if(highVal >= ldrGoal){
    robot.go(0,0);
  }
}
*/
//^End of photovore code

//Start of robot fence code
void rover() {
  leftIRdata = digitalRead(leftIR); //read the left IR 
  rightIRdata = digitalRead(rightIR); //read the right IR 
  //it veers left when going forward/straight - adjust (increase) left motor value to compensate
  // variant options: attempt nested while/if loop combos, switch case format, switch to analog pins for IR for range of data feedback, combine statements with conditional opperands in (parens)
  int highVal = getLargest();

  if(highVal == -1){
    robot.go(0,0);
  }
  else if(highVal >= ldrGoal){
    robot.go(0,0);
  }
  else if ((leftIRdata == 0 && rightIRdata == 0)&&(highVal == data[1])){
    robot.go(90,85);
    delay(150);
    robot.stopMotors();
    delay(100);
    Serial.println("going forward");
  }
  else if (leftIRdata != 0 || rightIRdata != 0){
    robot.go(-100, -100);
    Serial.println("going backward");
    delay(600);
    robot.go(-100,100);
    Serial.println("turning");
    delay(500);
    robot.stopMotors();
  }
  else if (leftIRdata != 0 && rightIRdata != 0){
    robot.go(-100, -100);
    Serial.println("going backward");
    delay(600);
    robot.go(-100,100);
    Serial.println("turning");
    delay(500);
    robot.stopMotors();
  }
  /*
  else if (highVal == data[1]){
    robot.go(150,100);
  }
  */
  else if(highVal == data[0]){
    robot.go(-150,100);
  }
  else if(highVal == data[2]){
    robot.go(150,-100);
  }
}


void loop() {
  // put your main code here, to run repeatedly:
  rover();
}
