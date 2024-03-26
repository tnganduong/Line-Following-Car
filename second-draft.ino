// Motor A - Left Motor
#define enL 5
#define inL1 22
#define inL2 24
// Motor B - Right Motor
#define enR 4
#define inR1 26
#define inR2 28
int status = 0;
int count = 0;
// PID
int P = 0;
int I = 0;
int D = 0;
int error = 0;
int lastError = 0;
float Kp = 30;
float Ki = 0;
float Kd = 0.5;
//Encoder
int encoderPinR1 = 18;  //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPinR2 = 19;  //Encoder Otput 'B' must connected with intreput pin of arduino.
int encoderPinL1 = 2;
int encoderPinL2 = 3;
volatile int lastEncodedL = 0;  // Here updated value of encoder store.
volatile int lastEncodedR = 0;

volatile long encoderValueL = 0;  // Raw encoder value
volatile long encoderValueR = 0;

// const int K = 37.5;

int in1 = 8;
int in2 = 9;
int in3 = 10;
int in4 = 11;
int in5 = 12;

void setup() {
  Serial.setTimeout(1);
  Serial.begin(9600);  //initialize serial comunication
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);


  pinMode(encoderPinR1, INPUT_PULLUP);
  pinMode(encoderPinR2, INPUT_PULLUP);
  pinMode(encoderPinL1, INPUT_PULLUP);
  pinMode(encoderPinL2, INPUT_PULLUP);

  pinMode(in1, INPUT);
  pinMode(in2, INPUT);
  pinMode(in3, INPUT);
  pinMode(in4, INPUT);
  pinMode(in5, INPUT);

  digitalWrite(encoderPinR1, HIGH);  //turn pullup resistor on
  digitalWrite(encoderPinR2, HIGH);  //turn pullup resistor on
  digitalWrite(encoderPinL1, HIGH);  //turn pullup resistor on
  digitalWrite(encoderPinL2, HIGH);  //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen on
  // interrupt # 5, pin 18 or interrupt # 4, pin 19
  attachInterrupt(5, updateEncoderR, CHANGE);
  attachInterrupt(4, updateEncoderR, CHANGE);
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3)
  attachInterrupt(0, updateEncoderL, CHANGE);
  attachInterrupt(1, updateEncoderL, CHANGE);
  
  delay(5000);
}

void loop() {
  // Serial.println("Running");
  //Read IR sensor pin
  int s1 = digitalRead(in1);
  int s2 = digitalRead(in2);
  int s3 = digitalRead(in3);
  int s4 = digitalRead(in4);
  int s5 = digitalRead(in5);

  // stop();
  // 1 == white
  // 0 == black
  String arduino = Serial.readString();
  s1 = digitalRead(in1);
  s2 = digitalRead(in2);
  s3 = digitalRead(in3);
  s4 = digitalRead(in4);
  s5 = digitalRead(in5);
  if (s1 == 1 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0) {
      // error = -4;
      rotate_left_90();
  } else if (s1 == 1 && s2 == 1 && s3 == 0 && s4 == 0 && s5 == 0){
    // error = -4;
    // PID_control();
    rotate_left_90();
  } else if (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 0 && s5 == 0){
    error = -4;
    PID_control();
    // rotate_left1();
  } else if (s1 == 0 && s2 == 1 && s3 == 0 && s4 == 0 && s5 == 0){
    // error = -4;
    // PID_control();
    rotate_left_45();
  } else if (s1 == 0 && s2 == 1 && s3 == 1 && s4 == 0 && s5 == 0){
    error = -2;
    PID_control();
  } else if (s1 == 0 && s2 == 0 && s3 == 1 && s4 == 0 && s5 == 0){
    error = 0;
    PID_control();
  } else if (s1 == 0 && s2 == 0 && s3 == 1 && s4 == 1 && s5 == 0){
    error = 2;
    PID_control();
  } else if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 1 && s5 == 0){
    // error = 4;
    // PID_control();
    rotate_right_90();
  } else if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 1 && s5 == 1){
    // error = 4;
    // PID_control();
    rotate_right_90();
  } else if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 1){
    rotate_right_90();
  }
    else if (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1){
      stop();delay(2000);
      camera();
}
      
}
void updateEncoderL() {
  int MSBL = digitalRead(encoderPinL1);  //MSB = most significant bit
  int LSBL = digitalRead(encoderPinL2);  //LSB = least significant bit

  int encodedL = (MSBL << 1) | LSBL;          //converting the 2 pin value to single number
  int sumL = (lastEncodedL << 2) | encodedL;  //adding it to the previous encoded value

  if (sumL == 0b1101 || sumL == 0b0100 || sumL == 0b0010 || sumL == 0b1011) encoderValueL--;  //clockwise movement
  if (sumL == 0b1110 || sumL == 0b0111 || sumL == 0b0001 || sumL == 0b1000) encoderValueL++;  //counter-clockwise movement
  lastEncodedL = encodedL;
}
void updateEncoderR() {
  int MSBR = digitalRead(encoderPinR1);  //MSB = most significant bit
  int LSBR = digitalRead(encoderPinR2);  //LSB = least significant bit

  int encodedR = (MSBR << 1) | LSBR;          //converting the 2 pin value to single number
  int sumR = (lastEncodedR << 2) | encodedR;  //adding it to the previous encoded value

  if (sumR == 0b1101 || sumR == 0b0100 || sumR == 0b0010 || sumR == 0b1011) encoderValueR--;  //clockwise movement
  if (sumR == 0b1110 || sumR == 0b0111 || sumR == 0b0001 || sumR == 0b1000) encoderValueR++;  //counter-clockwise movement
  lastEncodedR = encodedR;
}
void stop() {
  digitalWrite(enR, 0);
  digitalWrite(enL, 0);
  digitalWrite(inR1, 0);
  digitalWrite(inR2, 0);
  digitalWrite(inL1, 0);
  digitalWrite(inL2, 0);
}

void goForward(int speedR, int speedL) {

  // Turn on motor A & B
  digitalWrite(inL1, 1);
  digitalWrite(inL2, 0);
  digitalWrite(inR1, 1);
  digitalWrite(inR2, 0);

  analogWrite(enR, speedR);
  analogWrite(enL, speedL);
  // Serial.println("Left");
  // Serial.println(speedL);
  // Serial.println("Right");
  // Serial.println(speedR);
}
void PID_control() {
  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P * Kp + I * Ki + D * Kd;  //calculate the correction needed to be applied to the speed
  int motorspeedL = 120 + motorspeed;         // basespeed = 130
  int motorspeedR = 120 - motorspeed;

  if (motorspeedL > 255) {
    motorspeedL = 255;
  }
  if (motorspeedR > 255) {
    motorspeedR = 255;
  }
  if (motorspeedL < 120) {
    motorspeedL = 120;
  }
  if (motorspeedR < 120) {
    motorspeedR = 120;
  }

  goForward(motorspeedR, motorspeedL);
}
void rotate_right_90() {
  int speed = 255;
  digitalWrite(inR1, 0);
  digitalWrite(inR2, 1);
  digitalWrite(inL1, 1);
  digitalWrite(inL2, 0);

  analogWrite(enR, speed);
  analogWrite(enL, speed);
}
void rotate_right_45() {
  int speed = 255;
  digitalWrite(inR1, 0);
  digitalWrite(inR2, 1);
  digitalWrite(inL1, 1);
  digitalWrite(inL2, 0);

  analogWrite(enR, speed);
  analogWrite(enL, speed);
}

void rotate_left_90() {
  int speed = 255;  // 150
  digitalWrite(inR1, 1);
  digitalWrite(inR2, 0);
  digitalWrite(inL1, 0);
  digitalWrite(inL2, 1);

  analogWrite(enR, speed);
  analogWrite(enL, speed);
}
void rotate_left_45() {
  int speed = 255;
  digitalWrite(inR1, 1);
  digitalWrite(inR2, 0);
  digitalWrite(inL1, 0);
  digitalWrite(inL2, 1);

  analogWrite(enR, speed);
  analogWrite(enL, speed);
}
void camera (){
  String arduino = Serial.readString();
     if (arduino = "c") {
      line_following();
      delay(6000);
      stop();
      delay(1000);
      rotate_right_90();
      delay(500);
      line_following();
      delay(1000);
    }
      else if (arduino = "d") {
      line_following();
      delay(6000);
      stop();
      delay(1000);
      rotate_left_90();
      delay(1000);
      line_following();
      delay(1000);
    }
      else if (arduino = "e") {
      stop();
      delay(5000);
    }
}
void line_following () {
  int s1 = digitalRead(in1);
  int s2 = digitalRead(in2);
  int s3 = digitalRead(in3);
  int s4 = digitalRead(in4);
  int s5 = digitalRead(in5);

  if (s1 == 1 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0) {
      // error = -4;
      rotate_left_90();
  } else if (s1 == 1 && s2 == 1 && s3 == 0 && s4 == 0 && s5 == 0){
    // error = -4;
    // PID_control();
    rotate_left_90();
  } else if (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 0 && s5 == 0){
    error = -4;
    PID_control();
    // rotate_left1();
  } else if (s1 == 0 && s2 == 1 && s3 == 0 && s4 == 0 && s5 == 0){
    // error = -4;
    // PID_control();
    rotate_left_45();
  } else if (s1 == 0 && s2 == 1 && s3 == 1 && s4 == 0 && s5 == 0){
    error = -2;
    PID_control();
  } else if (s1 == 0 && s2 == 0 && s3 == 1 && s4 == 0 && s5 == 0){
    error = 0;
    PID_control();
  } else if (s1 == 0 && s2 == 0 && s3 == 1 && s4 == 1 && s5 == 0){
    error = 2;
    PID_control();
  } else if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 1 && s5 == 0){
    // error = 4;
    // PID_control();
    rotate_right_90();
  } else if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 1 && s5 == 1){
    // error = 4;
    // PID_control();
    rotate_right_90();
  } else if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 1){
    rotate_right_90();
  }
}
