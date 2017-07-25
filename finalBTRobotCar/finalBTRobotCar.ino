// LIBRARIES NEEDED
#include <AFMotor.h>   // FOR THE MOTOR SHIELD
#include <Servo.h>     // FOR MICRO SERVO
#include <NewPing.h>    // FOR ULTRASONIC PING SENSOR

#define TRIG_PIN 14 // Pin A4 on the Motor Drive Shield used for Ping Sensor
#define ECHO_PIN 15 // Pin A5 on the Motor Drive Shield used for Ping Sensor
#define MAX_DISTANCE 200 // sets maximum useable sensor measuring distance to 200cm
#define MAX_SPEED 180 // sets speed of DC traction motors to 180/256 or about 70% of full speed
#define MAX_SPEED_OFFSET 10 // sets offset to allow for differences between the 4 DC traction motors
#define COLL_DIST 10 // sets distance at which robot stops and reverses to 10cm
#define TURN_DIST COLL_DIST+10 // sets distance at which robot veers away from object (not reverse) to 20cm (10+10)

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); // sets up sensor library

AF_DCMotor motor1(1, MOTOR12_1KHZ); // create motor #1 using M1 output on Motor Drive Shield, set to 1kHz PWM frequency
AF_DCMotor motor2(2, MOTOR12_1KHZ); // create motor #2, using M2 output, set to 1kHz PWM frequency
AF_DCMotor motor3(3, MOTOR34_1KHZ); // create motor #3, using M2 output, set to 1kHz PWM frequency
AF_DCMotor motor4(4, MOTOR34_1KHZ); // create motor #4, using M2 output, set to 1kHz PWM frequency

Servo myservo;  // create servo object to control a servo 

int pos = 0; // sets up variables for use in the sketch (code)
int maxDist = 0;
int maxAngle = 0;
int maxRight = 0;
int maxLeft = 0;
int maxFront = 0;
int course = 0;
int curDist = 0;
String motorSet = "";
int speedSet = 0;

//-------------------------------------------- SETUP LOOP ----------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);   // for bluetooth signals
  
  myservo.attach(10);    // attaches the servo on pin 9 (SERVO_1 on the Motor Drive Shield to the servo object) 
  myservo.write(90);     // tells the servo to position at 90-degrees ie. facing forward.
  delay(2000);           // delay for two seconds
  checkPath();           // run the CheckPath routine to find the best path to begin travel
  motorSet = "FORWARD";  // set the director indicator variable to FORWARD
  myservo.write(90);     // make sure servo is still facing forward
  moveForward();         // run function to make robot move forward
}
//------------------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------MAIN LOOP ------------------------------------------------------------------------------
void loop() {
  if (Serial.available() > 0) {
      char ch = Serial.read();
      Serial.print("Received: ");
      Serial.println(ch);
      if (ch == 'q') {                // give control to bluetooth if 'q' char sent
          moveStop();
          delay(100);
          ch = Serial.read();
          int safeDistance;
          while( ch!= 'w'){           // break out of bluetooth control if 'w' char sent
              safeDistance = readPing();    // check to see if no obstacle in front
              if( ch == 'f'){    
                  if(safeDistance > COLL_DIST){
                      moveForward();
                  }else {
                      checkCourse();
                  }
              }else if( ch == 'b'){
                  moveBackward();
              }else if( ch == 'r'){
                if(safeDistance > COLL_DIST){
                      turnRight();
                  }else {
                      checkCourse();
                  }
              }else if( ch == 'l'){
                  if(safeDistance > COLL_DIST){
                      turnLeft();
                  }else {
                      checkCourse();
                  }
              }
              ch = Serial.read();
          }
        }
      }else{
          checkForward(); // check that if the robot is supposed to be moving forward, that the drive motors are set to move forward - 
          checkPath(); // set ultrasonic sensor to scan for any possible obstacles
      }
}
//--------------------------------------------------------------------------------------------------------------------------------------

// rotate the servo and check path to make sure obstacles are avoided
void checkPath() {
  int curLeft = 0;
  int curFront = 0;
  int curRight = 0;
  int curDist = 0;
  myservo.write(144); // set servo to face left 54-degrees from forward
  delay(120);         // wait 120 milliseconds for servo to reach position
  for(pos = 144; pos >= 36; pos-=18)     // loop to sweep the servo (& sensor) from 144-degrees left to 36-degrees right at 18-degree intervals. 
  {
    myservo.write(pos);  // tell servo to go to position in variable 'pos' 
    delay(90); // wait 90ms for servo to get to position   
    checkForward(); // check the robot is still moving forward
    curDist = readPing(); // get the current distance to any object in front of sensor
    if (curDist < COLL_DIST) { // if the current distance to object is less than the collision distance
      checkCourse(); // run the checkCourse function
      break; // jump out of this loop
    }
    if (curDist < TURN_DIST) { // if current distance is less than the turn distance
      changePath(); // run the changePath function
    }
    if (curDist > curDist) {maxAngle = pos;}
    if (pos > 90 && curDist > curLeft) { curLeft = curDist;}
    if (pos == 90 && curDist > curFront) {curFront = curDist;}
    if (pos < 90 && curDist > curRight) {curRight = curDist;}
  }
  maxLeft = curLeft;
  maxRight = curRight;
  maxFront = curFront;
}

// set direction for travel
void setCourse() { 
    if (maxAngle < 90) {turnRight();}
    if (maxAngle > 90) {turnLeft();}
    maxLeft = 0;
    maxRight = 0;
    maxFront = 0;
}

// we're about to hit something so move backwards, stop, find where the empty path is.
void checkCourse() { 
  moveBackward();
  delay(500);
  moveStop();
  setCourse();
}

void changePath() {
  if (pos < 90) {veerLeft();} // if current pos of sensor is less than 90-degrees, it means the object is on the right hand side so veer left
  if (pos > 90) {veerRight();} // if current pos of sensor is greater than 90-degrees, it means the object is on the left hand side so veer right
}

int readPing() { // read the ultrasonic sensor distance
  delay(70);
  unsigned int uS = sonar.ping();
  int cm = uS/US_ROUNDTRIP_CM;
  return cm;
}
// make sure motors are going forward
void checkForward() { 
  if (motorSet=="FORWARD") {
      motor1.run(FORWARD); 
      motor4.run(FORWARD);
      motor2.run(FORWARD);
      motor3.run(FORWARD); 
    }
}     
// make sure motors are going backward
void checkBackward() { 
  if (motorSet=="BACKWARD") {
      motor3.run(BACKWARD);
      motor2.run(BACKWARD);
      motor4.run(BACKWARD);
      motor1.run(BACKWARD); 
    } 
} 


// stop the motors
void moveStop() {
  motor1.run(RELEASE); 
  motor4.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
}  
// move Forward
void moveForward() {
    motorSet = "FORWARD";
    motor1.run(FORWARD);      // turn it on going forward
    motor4.run(FORWARD);
    motor2.run(FORWARD);      // turn it on going forward
    motor3.run(FORWARD);
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2) // slowly bring the speed up
  {
    motor1.setSpeed(speedSet+MAX_SPEED_OFFSET);
    motor4.setSpeed(speedSet+MAX_SPEED_OFFSET);
    motor2.setSpeed(speedSet);
    motor3.setSpeed(speedSet);
    delay(5);
  }
}

void moveBackward() {
    motorSet = "BACKWARD";
    motor1.run(BACKWARD);      // turn it on going forward
    motor4.run(BACKWARD);
    motor2.run(BACKWARD);    // turn it on going forward
    motor3.run(BACKWARD);
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2) // slowly bring the speed up
  {
    motor2.setSpeed(speedSet+MAX_SPEED_OFFSET);
    motor3.setSpeed(speedSet+MAX_SPEED_OFFSET);
    motor4.setSpeed(speedSet);
    motor1.setSpeed(speedSet);
    delay(5);
  }
}  

void turnRight() {
  motorSet = "RIGHT";
  motor1.run(FORWARD);      // turn motor 1 & 2 forward
  motor2.run(FORWARD);     
  motor4.run(BACKWARD);      // turn motor 4 & 3 backward
  motor3.run(BACKWARD);
  delay(400);               // run motors this way for 400ms
  motorSet = "FORWARD";
  
  // set all motors back to forward
  motor1.run(FORWARD);      
  motor4.run(FORWARD);
  motor3.run(FORWARD);
  motor2.run(FORWARD);
}  
//-------------------------------------------------------------------------------------------------------------------------------------
void turnLeft() {
  motorSet = "LEFT";
  motor1.run(BACKWARD);     // turn motor 1 backward
  motor2.run(BACKWARD);
  motor4.run(FORWARD);      // turn motor 2 forward
  motor3.run(FORWARD);
  delay(400);               // run motors this way for 400ms
  motorSet = "FORWARD";
  // set all motors back to forward
  motor1.run(FORWARD);      // turn it on going forward
  motor4.run(FORWARD);      // turn it on going forward
  motor2.run(FORWARD);
  motor3.run(FORWARD);
}  

void veerRight() {
  motor4.run(BACKWARD);
  motor3.run(BACKWARD); 
  delay(400);                // set right motor backwards for 400ms
  motor4.run(FORWARD);
  motor3.run(FORWARD);
} 

void veerLeft() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD); 
  delay(400);                // set left motor backwards for 400ms
  motor1.run(FORWARD);
  motor2.run(FORWARD);
} 




