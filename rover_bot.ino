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

const int DISTANCE_TO_TURN = 35; //cm

void setup(){
  delay(1000);
  Serial.begin(9600);
  Serial.println("Motor test");
  pinMode(ULTRASONIC_TRIG,  OUTPUT);
  pinMode(ULTRASONIC_ECHO,  INPUT);
  pinMode(RIGHT_MOTOR_EN,       OUTPUT);
  pinMode(RIGHT_MOTOR_POS,      OUTPUT);
  pinMode(RIGHT_MOTOR_NEG,      OUTPUT);
  pinMode(LEFT_MOTOR_EN,       OUTPUT);
  pinMode(LEFT_MOTOR_POS,      OUTPUT);
  pinMode(LEFT_MOTOR_NEG,      OUTPUT);
  roverSpeed(150);
  roverStop();
}


void loop(){
  long distance;
  static int reroute_count = 0;
  distance = getUltrasonicDistance();
  if (distance < DISTANCE_TO_TURN){
    if (reroute_count > 3) {
      Serial.println((String)"Can't avoid obstacle, trying something different");
      roverReverse();
      delay(2000);
      roverLeft();
      delay(500);
    }
    roverReverse();
    delay(200);
    roverRight();
    delay(200);
    reroute_count++;
    Serial.println((String)"Distance from rover is " + distance + " CM");
  } else {
    reroute_count = 0;
    roverForward();
    Serial.println("No obstacle detected.  Going forward");
    delay(15);
  }
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

void right_forward(){
  digitalWrite(RIGHT_MOTOR_POS, HIGH);
  digitalWrite(RIGHT_MOTOR_NEG, LOW);
}

void right_stop(){
  digitalWrite(RIGHT_MOTOR_POS, LOW);
  digitalWrite(RIGHT_MOTOR_NEG, LOW);
}

void right_backwards(){
  digitalWrite(RIGHT_MOTOR_POS, LOW);
  digitalWrite(RIGHT_MOTOR_NEG, HIGH);
}

void left_forward(){
  digitalWrite(LEFT_MOTOR_POS, HIGH);
  digitalWrite(LEFT_MOTOR_NEG, LOW);
}

void left_stop(){
  digitalWrite(LEFT_MOTOR_POS, LOW);
  digitalWrite(LEFT_MOTOR_NEG, LOW);
}

void left_backwards(){
  digitalWrite(LEFT_MOTOR_POS, LOW);
  digitalWrite(LEFT_MOTOR_NEG, HIGH);
}


void roverForward(){
  right_forward();
  left_forward();
}

void roverRight(){
  right_forward();
  left_backwards();
}

void roverLeft(){
  right_backwards();
  left_forward();
}

void roverReverse(){
  right_backwards();
  left_backwards();
}

void roverStop(){
  left_stop();
  right_stop();
}


