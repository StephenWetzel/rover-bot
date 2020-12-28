 /*
  Creation Crate DIY Rover Bot!
  This is an obstacle avoidance rover. It works by using ultrasonic obstacle/distance sensor. 
  Similar to a bat's echolocation ability, it can detect object by bouncing a signal off of them.
  The rover will move forward if there is no obstacle within 25cm. Once an object is detected, it 
  will turn right until the coast is clear and keep on truckin'.
  
  Components Required
  1. UNO R3 Microcontroller
  2. DC Motor Driver L298N
  3. HC-SR04 Ultrasonic Distance sensor

  The circuit:
    Right Motor 1: Positive to Out1, negative to Out2
    Left Motor 2: Positive to Out3, negative to Out4
    Driver Board control: ENA - 3 | IN1 - 8 | IN2 - 7 | IN3 - 5 | IN4 - 4 | ENB - 6
    Ultrasonic Sensor: TRIG - A4 | ECHO - A5

  Inputs and Motor Movement
  
  ################################
  # IN1  # IN2  # MOTOR          #
  ################################
  # LOW  # LOW  # STOP           #
  # LOW  # HIGH # CLOCKWISE      #
  # HIGH # LOW  # COUNTER-CW     #
  # HIGH # HIGH # STOP           #
  ################################
  
  Motor & Rover Movement
  
  ###########################################################
  # LEFT MOTOR 1       # RIGHT MOTOR 2     # ROVER MOVEMENT #
  ###########################################################
  # CLOCKWISE          # CLOCKWISE         # FORWARD        #
  # COUNTER CLOCKWISE  # COUNTER CLOCKWISE # BACKWARD       #
  # CLOCKWISE          # COUNTER CLOCKWISE # TURNS LEFT     #
  # COUNTER CLOCKWISE  # CLOCKWISE         # TURNS RIGHT    #
  # OFF (RELEASE)      # OFF (RELEASE)     # STOP           #
  ###########################################################

  Functions
  As discussed in the last project, functions are usually created for tasks that are needed 
  throughout the program so we don't have to write the same instructions over and over.
    roverStop, roverForward, roverReverse, roverLeft, & roverRight - directional commands
    getUltrasonicDistance - returns ultrasonic distance in centimeters  

  ULTRASONIC
  Here we are using ultrasonic distance sensor to find distance of obstacle.
  Basically this sensor has a transmitter and a receiver.
  1. Upon TRIGGER transmitter sends ULTRASONIC BURST 
  2. ULTRASONIC waves will hit the obstacle and reflect back
  3. Receiver in the sensor will sense the reflected signals and calculate time
  4. Remember the famous equation: distance = speed * time, speed of ultrasonic signals is same 
      as of sound signals, thus we can calculate the distance based on it. 

  Created 08.01.18
  By Ligo George & David Hehman
  http://creationcrate.com/
 */

#define ULTRASONIC_TRIG   A5
#define ULTRASONIC_ECHO   A4
#define RIGHT_MOTOR_EN         3
#define RIGHT_MOTOR_POS        5
#define RIGHT_MOTOR_NEG        4
#define LEFT_MOTOR_EN          6
#define LEFT_MOTOR_POS         7
#define LEFT_MOTOR_NEG         8

const int DISTANCE_TO_TURN = 25; //cm
const int DISTANCE_TO_VEER = 80; //cm

void setup(){
  delay(1000);
  Serial.begin(9600);
  Serial.println("Motor test");
  pinMode(ULTRASONIC_TRIG,  OUTPUT);
  pinMode(ULTRASONIC_ECHO,  INPUT);
  pinMode(RIGHT_MOTOR_EN,   OUTPUT);
  pinMode(RIGHT_MOTOR_POS,  OUTPUT);
  pinMode(RIGHT_MOTOR_NEG,  OUTPUT);
  pinMode(LEFT_MOTOR_EN,    OUTPUT);
  pinMode(LEFT_MOTOR_POS,   OUTPUT);
  pinMode(LEFT_MOTOR_NEG,   OUTPUT);
  roverSpeed(150);
  roverStop();
}


void loop(){
  // speedTest();
  long distance;
  static int reroute_count = 0;
  distance = getUltrasonicDistance();
  Serial.println((String)"Distance from rover is " + distance + " CM");
  if (distance < DISTANCE_TO_TURN) {
    avoidObstacle(reroute_count);
    reroute_count++;
  } else if (distance < DISTANCE_TO_VEER) {
    approachingObstacle();
  } else {
    reroute_count = 0;
    roverMed();
    roverForward();
    Serial.println("No obstacle detected.  Going forward");
    delay(15);
  }
}

void avoidObstacle(int reroute_count) {
  roverSlow();
  if (reroute_count > 10) {
    avoidProtocol3();
  } else if (reroute_count > 3) {
    avoidProtocol2();
  } else {
    avoidProtocol1();
  }
}

void avoidProtocol1() {
  long distance;
  int turn_count = 0;
  Serial.println((String)"Implementing avoid protocol 1");
  roverReverse();
  delay(150);

  distance = getUltrasonicDistance();
  while (turn_count < 25 && distance < DISTANCE_TO_TURN) {
    turn_count++;
    roverRight();
    delay(100);
    distance = getUltrasonicDistance();
  }
}

void avoidProtocol2() {
  Serial.println((String)"Implementing avoid protocol 2");
  roverReverse();
  delay(2000);
  roverLeft();
  delay(500);
}

void avoidProtocol3() {
  Serial.println((String)"Implementing avoid protocol 3");
  roverReverse();
  delay(2000);
  roverLeft();
  delay(500);
  roverRight();
  delay(500);
  roverLeft();
  delay(500);
  roverRight();
  delay(500);

  roverFast();
  roverLeft();
  delay(500);
  roverRight();
  delay(500);
  roverReverse();
  delay(2000);
}

void approachingObstacle() {
  roverVeerRight();
  delay(15);
}

void roverSpeed(int speed){
  analogWrite(RIGHT_MOTOR_EN, speed);
  analogWrite(LEFT_MOTOR_EN, speed);
}

long getUltrasonicDistance(){
  long duration;
  long distance;
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  duration = pulseIn(ULTRASONIC_ECHO, HIGH);
  Serial.println((String)"duration: " + duration);
  distance = (duration / 2) / 29.1;
  Serial.println((String)"distance: " + distance);

  return distance;
}

// individual right motor controls:
void rightSlow() {
  analogWrite(RIGHT_MOTOR_EN, 110);
}

void rightMed() {
  analogWrite(RIGHT_MOTOR_EN, 180);
}

void rightFast() {
  analogWrite(RIGHT_MOTOR_EN, 255);
}

void rightForward(){
  digitalWrite(RIGHT_MOTOR_POS, HIGH);
  digitalWrite(RIGHT_MOTOR_NEG, LOW);
}

void rightStop(){
  digitalWrite(RIGHT_MOTOR_POS, LOW);
  digitalWrite(RIGHT_MOTOR_NEG, LOW);
}

void rightBackwards(){
  digitalWrite(RIGHT_MOTOR_POS, LOW);
  digitalWrite(RIGHT_MOTOR_NEG, HIGH);
}

// individual left motor controls:
void leftSlow() {
  analogWrite(LEFT_MOTOR_EN, 110);
}

void leftMed() {
  analogWrite(LEFT_MOTOR_EN, 180);
}

void leftFast() {
  analogWrite(LEFT_MOTOR_EN, 255);
}

void leftForward(){
  digitalWrite(LEFT_MOTOR_POS, HIGH);
  digitalWrite(LEFT_MOTOR_NEG, LOW);
}

void leftStop(){
  digitalWrite(LEFT_MOTOR_POS, LOW);
  digitalWrite(LEFT_MOTOR_NEG, LOW);
}

void leftBackwards(){
  digitalWrite(LEFT_MOTOR_POS, LOW);
  digitalWrite(LEFT_MOTOR_NEG, HIGH);
}

// overall rover controls:
void roverForward(){
  rightForward();
  leftForward();
}

void roverRight(){
  rightForward();
  leftBackwards();
}

void roverLeft(){
  rightBackwards();
  leftForward();
}

void roverReverse(){
  rightBackwards();
  leftBackwards();
}

void roverStop(){
  leftStop();
  rightStop();
}

void roverVeerRight() {
  rightFast();
  leftSlow();
  roverForward();
}

void roverVeerLeft() {
  rightSlow();
  leftFast();
  roverForward();
}

void roverSlow() {
  rightSlow();
  leftSlow();
  roverForward();
}

void roverMed() {
  rightMed();
  leftMed();
  roverForward();
}

void roverFast() {
  rightSlow();
  leftSlow();
  roverForward();
}



void speedTest() {
  for (int ii = 0; ii <= 255; ii++) {
    Serial.println((String)"Speed is " + ii);
    analogWrite(RIGHT_MOTOR_EN, ii);
    analogWrite(LEFT_MOTOR_EN, ii);
    roverForward();
    delay(100);
    // 82 left
    //109 right
  }
}
