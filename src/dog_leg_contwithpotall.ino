#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <NewPing.h>
#include <math.h>

#define TRIG_PIN 9
#define ECHO_PIN 10
#define MAX_DISTANCE 200 // Maximum distance to ping (in cm)
#define SERVOMIN  150  // Minimum pulse length count (adjust this for your servos)
#define SERVOMAX  600  // Maximum pulse length count (adjust this for your servos)

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

// Initial angles for each servo (these will change based on the potentiometer)
int servo1_angle = 45;
int servo2_angle = 55;
int servo3_angle = 45;
int servo4_angle = 30;
int servo5_angle = 140;
int servo6_angle = 35;
int servo7_angle = 90; // Added servo 7
int servo8_angle = 60; // Added servo 8

// Potentiometer pin
const int potPin = A0; // Potentiometer connected to A0

int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

#define DIST_THRESHOLD 5  // Small change in distance that won’t trigger servo movements

void uls() {
  unsigned int dis = sonar.ping_cm();

  // Adjust servos based on the ultrasonic sensor reading if distance is below 55 cm
  if (dis <= 55) {
    pwm.setPWM(0, 0, angleToPulse(60));  // Servo 1
    pwm.setPWM(1, 0, angleToPulse(26));  // Servo 2
    pwm.setPWM(6, 0, angleToPulse(40));  // Servo 7
    pwm.setPWM(7, 0, angleToPulse(145)); // Servo 8

    delay(50);  // Short delay to stabilize
  }
}

//int servo1_angle, servo2_angle, servo3_angle, servo4_angle, servo5_angle, servo6_angle, servo7_angle, servo8_angle;

// Define the lengths of the robot leg segments
float a = 12.2;  // Thigh length
float b = 11.5;  // Shin length

// Foot target position
float x = 8.0;
float y = 4.0;




void controlServosWithPotentiometer() {
  int potValue = analogRead(potPin);

  // Map potentiometer values to servo angles
  servo3_angle = map(potValue, 0, 1023, 25, 65);   // Servo 3 sweeps from 25 to 65
  servo4_angle = map(potValue, 0, 1023, 50, 0);    // Servo 4 sweeps from 50 to 0
  servo5_angle = map(potValue, 0, 1023, 170, 128); // Servo 5 sweeps from 170 to 128
  servo6_angle = map(potValue, 0, 1023, -5, 45);   // Servo 6 sweeps from -5 to 52

  // Set PWM for each servo
  pwm.setPWM(2, 0, angleToPulse(servo3_angle));  // Servo 3
  pwm.setPWM(3, 0, angleToPulse(servo4_angle));  // Servo 4
  pwm.setPWM(4, 0, angleToPulse(servo5_angle));  // Servo 5
  pwm.setPWM(5, 0, angleToPulse(servo6_angle));  // Servo 6

  // Rear legs controlled by potentiometer
  servo1_angle = map(potValue, 0, 1023, 25, 80);   // Servo 1 sweeps from 30 to 90
  servo2_angle = map(potValue, 0, 1023, 55, 8);    // Servo 2 sweeps from 53 to 0
  servo7_angle = map(potValue, 0, 1023, 65, 10);   // Servo 7 sweeps from 65 to 15
  servo8_angle = map(potValue, 0, 1023, 120, 170); // Servo 8 sweeps from 120 to 170

  pwm.setPWM(0, 0, angleToPulse(servo1_angle)); // Servo 1
  pwm.setPWM(1, 0, angleToPulse(servo2_angle)); // Servo 2
  pwm.setPWM(6, 0, angleToPulse(servo7_angle)); // Servo 7
  pwm.setPWM(7, 0, angleToPulse(servo8_angle)); // Servo 8
}

void setup() {
  Serial.begin(9600);

  // Initialize the PCA9685
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz frequency

  delay(10);  // Short delay to allow driver to stabilize
}




// Function to map torso-to-ground angles to servos

void controlLegsWithTorsoGround(int c) {
  if (c>20){
    c=18;
  }
  else if (c<8){
    c=8.5;
  }

  int thigh_ang, knee_ang;

  // Calculate angles using inverse kinematics equations
  thigh_ang = abs(90- (acos(((a * a) + (c * c) - (b * b)) / (2 * a * c)) * (180 / M_PI)));
  knee_ang = abs( 45-acos(((a * a) + (b * b) - (c * c)) / (2 * a * b)) * (180 / M_PI));

  // Inverted mappings for the front and rear legs
  // Front left leg
  servo1_angle = map(thigh_ang, 0, 90, 90, 25);  // Reverse map thigh angle to Servo 1
  servo2_angle = map(knee_ang, 0, 90, 0, 55);    // Reverse map knee angle to Servo 2
  pwm.setPWM(0, 0, angleToPulse(servo1_angle));  // Control Servo 1
  pwm.setPWM(1, 0, angleToPulse(servo2_angle));  // Control Servo 2
delay(200);
  // Front right leg
  servo3_angle = map(thigh_ang, 0, 90, 65, 25);  // Reverse map thigh angle to Servo 3
  servo4_angle = map(knee_ang, 0, 90, 0, 50);    // Reverse map knee angle to Servo 4
  pwm.setPWM(2, 0, angleToPulse(servo3_angle));  // Control Servo 3
  pwm.setPWM(3, 0, angleToPulse(servo4_angle));  // Control Servo 4
  //delay(50);
delay(200);
  // Rear left leg
  servo5_angle = map(thigh_ang, 0, 90, 128, 170); // Reverse map thigh angle to Servo 5
  servo6_angle = map(knee_ang, 0, 90, 45, 0);    // Reverse map knee angle to Servo 6
  pwm.setPWM(4, 0, angleToPulse(servo5_angle));   // Control Servo 5
  pwm.setPWM(5, 0, angleToPulse(servo6_angle));   // Control Servo 6
delay(200);
  // Rear right leg
  servo7_angle = map(thigh_ang, 0, 90, 15, 65);   // Reverse map thigh angle to Servo 7
  servo8_angle = map(knee_ang, 0, 90, 170, 120);  // Reverse map knee angle to Servo 8
  pwm.setPWM(6, 0, angleToPulse(servo7_angle));   // Control Servo 7
  pwm.setPWM(7, 0, angleToPulse(servo8_angle));   // Control Servo 8
}

void controlth(int c){
int cn;
int thigh_ang, knee_ang;
if (c>20){
    cn=18;
  }
  else if (c<8){
    cn=8.5;
  }


  

  // Calculate angles using inverse kinematics equations
  thigh_ang = abs(90- (acos(((a * a) + (c * c) - (b * b)) / (2 * a * c)) * (180 / M_PI)));
  knee_ang = abs( 45-acos(((a * a) + (b * b) - (c * c)) / (2 * a * b)) * (180 / M_PI));

  // Inverted mappings for the front and rear legs
  // Front left leg
  servo1_angle = map(thigh_ang, 0, 90, 90, 25);  // Reverse map thigh angle to Servo 1

  pwm.setPWM(0, 0, angleToPulse(servo1_angle));  // Control Servo 1
  
delay(200);
  // Front right leg
  servo3_angle = map(thigh_ang, 0, 90, 65, 25);  // Reverse map thigh angle to Servo 3
  
  pwm.setPWM(2, 0, angleToPulse(servo3_angle));  // Control Servo 3
 
  //delay(50);
delay(200);
  // Rear left leg
  servo5_angle = map(thigh_ang, 0, 90, 128, 170); // Reverse map thigh angle to Servo 5
  
  pwm.setPWM(4, 0, angleToPulse(servo5_angle));   // Control Servo 5

delay(200);
  // Rear right leg
  servo7_angle = map(thigh_ang, 0, 90, 15, 65);   // Reverse map thigh angle to Servo 7
 
  pwm.setPWM(6, 0, angleToPulse(servo7_angle));   // Control Servo 7
 

}

void motion(int x,int r){
int i =0;
int c,x1;
x1=1;
while(x1<x){
  c = sqrt((r*r)-(x1*x1));
  controlth(c);
  x1 = x1+0.1;
 

}

}


void walk (){
  int x,c,cn,th,knee_ang,r;
  x=-5;c=10;
  r=6;
  int i =0;
  while (i<3){
 
 th = ((atan(x/c)));
 th=(th*(180/M_PI));
 c = (c/cos(th));
 //controlth(c);


 


  knee_ang = abs((acos(((a * a) + (b * b) - (c * c)) / (2 * a * b)) * (180 / M_PI)))-25;
  th=70-knee_ang;
   //delay(350);

   delay(100);
  servo3_angle = map(th, 0, 90, 65, 25);  // Reverse map thigh angle to Servo 3
  servo4_angle = map(knee_ang, 0, 90, 0, 50);    // Reverse map knee angle to Servo 4
  pwm.setPWM(2, 0, angleToPulse(servo3_angle));  // Control Servo 3
  pwm.setPWM(3, 0, angleToPulse(servo4_angle));  // Control Servo 4
  delay(50);

  // Rear left leg


  // Rear right leg
  servo7_angle = map(th, 0, 90, 15, 65);   // Reverse map thigh angle to Servo 7
  servo8_angle = map(knee_ang, 0, 90, 170, 120);  // Reverse map knee angle to Servo 8
  pwm.setPWM(6, 0, angleToPulse(servo7_angle));   // Control Servo 7
  pwm.setPWM(7, 0, angleToPulse(servo8_angle));   // Control Servo 8
  delay(150);

  servo1_angle = map(th, 0, 90, 90, 25);  // Reverse map thigh angle to Servo 1
  servo2_angle = map(knee_ang, 0, 90, 0, 55);    // Reverse map knee angle to Servo 2
  pwm.setPWM(0, 0, angleToPulse(servo1_angle));  // Control Servo 1
  pwm.setPWM(1, 0, angleToPulse(servo2_angle));  // Control Servo 2
 delay(50);
  servo5_angle = map(th, 0, 90, 128, 170); // Reverse map thigh angle to Servo 5
  servo6_angle = map(knee_ang, 0, 90, 45, 0);    // Reverse map knee angle to Servo 6
  pwm.setPWM(4, 0, angleToPulse(servo5_angle));   // Control Servo 5
  pwm.setPWM(5, 0, angleToPulse(servo6_angle));   // Control Servo 6

  delay(150);


  
  
  c++;
  x=x+1;
  i++;

  }
  i--;

  while (i!=0){
 
 th = ((atan(x/c)));
 th=(th*(180/M_PI));
 c = (c/cos(th));
 //controlth(c);


 


  knee_ang = abs((acos(((a * a) + (b * b) - (c * c)) / (2 * a * b)) * (180 / M_PI)))-10;
  th=65-knee_ang;

  servo3_angle = map(th, 0, 90, 65, 25);  // Reverse map thigh angle to Servo 3
  servo4_angle = map(knee_ang, 0, 90, 0, 50);    // Reverse map knee angle to Servo 4
  pwm.setPWM(2, 0, angleToPulse(servo3_angle));  // Control Servo 3
  pwm.setPWM(3, 0, angleToPulse(servo4_angle));  // Control Servo 4
  delay(10);

  // Rear left leg


  // Rear right leg
  servo7_angle = map(th, 0, 90, 15, 65);   // Reverse map thigh angle to Servo 7
  servo8_angle = map(knee_ang, 0, 90, 170, 120);  // Reverse map knee angle to Servo 8
  pwm.setPWM(6, 0, angleToPulse(servo7_angle));   // Control Servo 7
  pwm.setPWM(7, 0, angleToPulse(servo8_angle));   // Control Servo 8
  delay(20);

   //delay(350);
  servo1_angle = map(th, 0, 90, 90, 25);  // Reverse map thigh angle to Servo 1
  servo2_angle = map(knee_ang, 0, 90, 0, 55);    // Reverse map knee angle to Servo 2
  pwm.setPWM(0, 0, angleToPulse(servo1_angle));  // Control Servo 1
  pwm.setPWM(1, 0, angleToPulse(servo2_angle));  // Control Servo 2
 delay(10);
  servo5_angle = map(th, 0, 90, 128, 170); // Reverse map thigh angle to Servo 5
  servo6_angle = map(knee_ang, 0, 90, 45, 0);    // Reverse map knee angle to Servo 6
  pwm.setPWM(4, 0, angleToPulse(servo5_angle));   // Control Servo 5
  pwm.setPWM(5, 0, angleToPulse(servo6_angle));   // Control Servo 6

  delay(20);


  
  
  c--;
  x=x-1;
  i--;

  }

  
 
}





void loop() {
  walk(); 
  //controlServosWithPotentiometer();

  if (Serial.available()) {
    String input = Serial.readString(); // Read the full string input initially
    input.trim(); // Remove any extra whitespace or newline characters

    if (input == "cp") { // Check if the input matches "CP"
      while (true) {
        if (Serial.available()) {
          String temp = Serial.readString();
          temp.trim();
          if (temp == "leave") break; // Exit the loop when "leave" is entered
        }
        controlServosWithPotentiometer();
      }
    } 
    else if (input == "ncp") { // For "NCP"
      while (true) {
        if (Serial.available()) {
          String temp = Serial.readString();
          temp.trim();
          if (temp == "leave") break; // Exit the loop when "leave" is entered
        }
        walk();
        //motion(10,20);
      }
    }
  }
}