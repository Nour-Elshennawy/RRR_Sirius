//SIRIUS ROVER CONTROL CODE//

#include <ESP32Servo.h>
#include <BTAddress.h>
#include <BTAdvertisedDevice.h>
#include <BTScan.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

// Define servo
#define SERVO_PIN 33
Servo servo1;

// Define IR sensor pins
#define IRS_LEFT 32
#define IRS_MIDDLE 35
#define IRS_RIGHT 34

// Define servo movement limits
#define MAX_LEFT 10
#define MAX_RIGHT 170
#define CENTER_STEER 90

// Define motors
#define MOT_R_IN1 22
#define MOT_R_IN2 21
#define MOT_L_IN3 19
#define MOT_L_IN4 18
#define ENA 23  //left
#define ENB 4   //right

// Define color sensor
#define S0 25
#define S1 26
#define out 27  //out pin
#define S2 14
#define S3 13

// variables
bool colorsensorstate = false;  // color sensor is off
bool AutonomousMode = false;    // start in autonomous mode

String data = "M";
String lastdir = "H";  // in the manual mode

int servoangle = CENTER_STEER;

int freq = 0;  // in color sensor mode

int motspeed = 150;        //initial motor speed
const int maxspeed = 255;  //max PWM
const int minspeed = 50;   //min safe speed

String last_turn;  // in the follow line mode

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_SIRIUS_ROVER");
  SerialBT.begin(115200);

  //MOTORS
  pinMode(MOT_R_IN1, OUTPUT);
  pinMode(MOT_R_IN2, OUTPUT);
  pinMode(MOT_L_IN3, OUTPUT);
  pinMode(MOT_L_IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  analogWrite(ENA, motspeed);
  analogWrite(ENB, motspeed);

  //SERVO
  servo1.attach(33);
  servo1.write(CENTER_STEER);  // start with centered steering

  //IR SENSORS
  pinMode(IRS_LEFT, INPUT);
  pinMode(IRS_MIDDLE, INPUT);
  pinMode(IRS_RIGHT, INPUT);

  //COLOR SENSOR
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(out, INPUT);
  // output freq scaling 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
}

// check for bluetooth commands and switch mode
void checkBluetooth() {
  if (SerialBT.available()) {
    data = SerialBT.readStringUntil(':');
    
    if (data == "A") {
      AutonomousMode = true;  // switch to autonomous mode

    } else if (data == "M") {
      AutonomousMode = false;  // switch to manual mode

    } else if (data == "O") {
      colorsensorstate = true;

    } else if (data == "C") {
      colorsensorstate = false;
    }
  } else data = "_";
}

// Main loop
void loop() {
  checkBluetooth();  // check for bluetooth commands

  if (AutonomousMode) {
    followLine();  // follow the line automatically
  } else {
    manualControl(data);  // control via Bluetooth
  }
  if (colorsensorstate) {
    colorsensormode();

    int red = getColor(LOW, LOW);
    int green = getColor(HIGH, HIGH);
    int blue = getColor(LOW, HIGH);

    SerialBT.print("color:");

    SerialBT.print(red);
    SerialBT.print('*');
    SerialBT.print(green);
    SerialBT.print('*');
    SerialBT.println(blue);

    delay(500);
  } else {
    colorsensormode();
  }

  //delay(10);  // small delay for stability
}

//follow the line autonomously
void followLine() {

  //IRS readings
  int left_read = digitalRead(IRS_LEFT);
  int middle_read = digitalRead(IRS_MIDDLE);
  int right_read = digitalRead(IRS_RIGHT);


  if (left_read == 1 && middle_read == 1 && right_read == 0)  //hard left
  {
    servo1.write(MAX_LEFT);
    leftIR();
    last_turn = "left";
  } else if (left_read == 1 && middle_read == 0 && right_read == 0)  //slight left
  {
    servo1.write(MAX_LEFT);
    leftIR();
    last_turn = "left";
  } else if (left_read == 0 && middle_read == 1 && right_read == 0)  //center
  {
    servo1.write(CENTER_STEER);
    forwardIR();
    last_turn = "center";
  } else if (left_read == 0 && middle_read == 0 && right_read == 1)  //slight rignt
  {
    servo1.write(MAX_RIGHT);
    rightIR();
    last_turn = "right";
  } else if (left_read == 0 && middle_read == 1 && right_read == 1)  //hard right
  {
    servo1.write(MAX_RIGHT);
    rightIR();
    last_turn = "right";
  } else if (left_read == 1 && middle_read == 1 && right_read == 1)  //continue moving when crossed lines
  {
    servo1.write(CENTER_STEER);
    forwardIR();
  } else if (left_read == 0 && middle_read == 0 && right_read == 0)  //continue moving when no lines
  {
    if (last_turn == "right") {
      servo1.write(MAX_RIGHT);
      rightIR();
    } else if (last_turn == "left") {
      servo1.write(MAX_LEFT);
      leftIR();
    } else if (last_turn == "center") {
      servo1.write(CENTER_STEER);
      forwardIR();
    }
  } else {
    forwardIR();  // always move with the last direction
  }
}

// bluetooth manual control
void manualControl(String command) {
  if (command == "L") {
    servoangle = max(servoangle - 10, MAX_LEFT);
    servo1.write(servoangle);
    left();
    delay(50);
  } else if (command == "R") {
    servoangle = min(servoangle + 10, MAX_RIGHT);
    servo1.write(servoangle);
    right();
    delay(50);
  } else if (command == "F") {
    forward();
    delay(50);
    lastdir = command;
  } else if (command == "B") {
    backward();
    delay(50);
    lastdir = command;
  } else if (command == "H") {
    stop();
    lastdir = command;
  } else if (command == "Z") {
    servoangle = CENTER_STEER;
    servo1.write(servoangle);
  } else if (command == "S" && motspeed < maxspeed) {
    motspeed += 25;
    if (motspeed >= maxspeed) { motspeed = maxspeed; }
    lastmovedir();
    delay(50);
  } else if (command == "D" && motspeed > minspeed) {
    motspeed -= 25;
    if (motspeed <= minspeed) { motspeed = minspeed; }
    lastmovedir();
    delay(50);
  } else stop();
}

// color sensor reading
int getColor(bool s2State, bool s3State) {
  digitalWrite(S2, s2State);
  digitalWrite(S3, s3State);
  return pulseIn(out, LOW);
}
//color sensor on/off
void colorsensormode() {
  if (colorsensorstate) {
    digitalWrite(S0, HIGH);
    digitalWrite(S1, LOW);
  } else {
    digitalWrite(S0, LOW);
    digitalWrite(S1, LOW);
  }
}

// control
void stop() {
  digitalWrite(MOT_R_IN1, LOW);
  digitalWrite(MOT_R_IN2, LOW);
  digitalWrite(MOT_L_IN3, LOW);
  digitalWrite(MOT_L_IN4, LOW);
}

void forward() {
  digitalWrite(MOT_R_IN1, HIGH);
  digitalWrite(MOT_R_IN2, LOW);
  digitalWrite(MOT_L_IN3, HIGH);
  digitalWrite(MOT_L_IN4, LOW);
  analogWrite(ENA, motspeed);
  analogWrite(ENB, motspeed);
}

void backward() {
  digitalWrite(MOT_R_IN1, LOW);
  digitalWrite(MOT_R_IN2, HIGH);
  digitalWrite(MOT_L_IN3, LOW);
  digitalWrite(MOT_L_IN4, HIGH);
  analogWrite(ENA, motspeed);
  analogWrite(ENB, motspeed);
}

void left() {
  if (lastdir == "F") {
    digitalWrite(MOT_R_IN1, HIGH);
    digitalWrite(MOT_R_IN2, LOW);
    digitalWrite(MOT_L_IN3, HIGH);
    digitalWrite(MOT_L_IN4, LOW);
    analogWrite(ENA, motspeed);
    analogWrite(ENB, motspeed - 50);
  } else if (lastdir == "B") {
    digitalWrite(MOT_R_IN1, LOW);
    digitalWrite(MOT_R_IN2, HIGH);
    digitalWrite(MOT_L_IN3, LOW);
    digitalWrite(MOT_L_IN4, HIGH);
    analogWrite(ENA, motspeed);
    analogWrite(ENB, motspeed - 50);
  } else if (lastdir == "H")
    stop();
}

void right() {
  if (lastdir == "F") {
    digitalWrite(MOT_R_IN1, HIGH);
    digitalWrite(MOT_R_IN2, LOW);
    digitalWrite(MOT_L_IN3, HIGH);
    digitalWrite(MOT_L_IN4, LOW);
    analogWrite(ENA, motspeed - 50);
    analogWrite(ENB, motspeed);
  } else if (lastdir == "B") {
    digitalWrite(MOT_R_IN1, LOW);
    digitalWrite(MOT_R_IN2, HIGH);
    digitalWrite(MOT_L_IN3, LOW);
    digitalWrite(MOT_L_IN4, HIGH);
    analogWrite(ENA, motspeed - 50);
    analogWrite(ENB, motspeed);
  } else if (lastdir == "H")
    stop();
}

void lastmovedir() {
  if (lastdir == "F") {
    forward();
  } else if (lastdir == "B") {
    backward();
  } else if (lastdir == "H") {
    stop();
  }
}

// autonomus
void forwardIR() {
  digitalWrite(MOT_R_IN1, HIGH);
  digitalWrite(MOT_R_IN2, LOW);
  digitalWrite(MOT_L_IN3, HIGH);
  digitalWrite(MOT_L_IN4, LOW);
  analogWrite(ENA, 55);
  analogWrite(ENB, 55);
}
void leftIR() {
  digitalWrite(MOT_R_IN1, LOW);
  digitalWrite(MOT_R_IN2, HIGH);
  digitalWrite(MOT_L_IN3, HIGH);
  digitalWrite(MOT_L_IN4, LOW);
  analogWrite(ENA, 115);
  analogWrite(ENB, 115);
}

void rightIR() {
  digitalWrite(MOT_R_IN1, HIGH);
  digitalWrite(MOT_R_IN2, LOW);
  digitalWrite(MOT_L_IN3, LOW);
  digitalWrite(MOT_L_IN4, HIGH);
  analogWrite(ENA, 115);
  analogWrite(ENB, 115);
}
