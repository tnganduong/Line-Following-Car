// Authors: Nguyen Huynh Duc, Duong Hoang Thanh Ngan
// Motor A - Left Motor
#define enL 5
#define inL1 22
#define inL2 24
// Motor B - Right Motor
#define enR 4
#define inR1 26
#define inR2 28
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
int encoderPinR1 = 18; 
int encoderPinR2 = 19;
int encoderPinL1 = 2;
int encoderPinL2 = 3;
// Here updated value of encoder store.
volatile int lastEncodedL = 0; 
volatile int lastEncodedR = 0;
// Raw encoder value
volatile long encoderValueL = 0; 
volatile long encoderValueR = 0;
// Sensors
int in1 = 8;
int in2 = 9;
int in3 = 10;
int in4 = 11;
int in5 = 12;

void setup() {
  Serial.begin(9600); //initialize serial comunication
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

  digitalWrite(encoderPinR1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPinR2, HIGH); //turn pullup resistor on
  digitalWrite(encoderPinL1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPinL2, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen on
  // interrupt # 5, pin 18 or interrupt # 4, pin 19
  attachInterrupt(5, updateEncoderR, CHANGE); 
  attachInterrupt(4, updateEncoderR, CHANGE);
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(0, updateEncoderL, CHANGE);
  attachInterrupt(1, updateEncoderL, CHANGE);
  
}

void loop() {
 
  //Read IR sensor pin
  int s1 = digitalRead(in1);
  int s2 = digitalRead(in2);
  int s3 = digitalRead(in3);
  int s4 = digitalRead(in4);
  int s5 = digitalRead(in5);
  // 1 == white
  // 0 == black
  
  if (s1 == 1 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0) {
    error = -4;
    PID_control();
    }
  else if (s1 == 1 && s2 == 1 && s3 == 0 && s4 == 0 && s5 == 0) {
    error = -4;
    PID_control();
    }
  else if (s1 == 0 && s2 == 1 && s3 == 0 && s4 == 0 && s5 == 0) {
    error = -4;
    PID_control();
    }
  else if (s1 == 0 && s2 == 1 && s3 == 1 && s4 == 0 && s5 == 0) {
    error = -2;
    PID_control();
    }
  else if (s1 == 0 && s2 == 0 && s3 == 1 && s4 == 0 && s5 == 0) {
    error = 0;
    PID_control();
    }
  else if (s1 == 0 && s2 == 0 && s3 == 1 && s4 == 1 && s5 == 0){
    error = 2;
    PID_control();
    }
  else if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 1 && s5 == 0){
    error = 4;
    PID_control();
    }
  else if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 1 && s5 == 1) {
    error = 4;
    PID_control();
    }
  else if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 1) {
    error = 4;
    PID_control();
    }
  else if (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1) {
    stop();
    delay(1000);
    to_goal();
    delay(16000);
    stop();
    delay(15000);
    }
   
} 

void updateEncoderL(){
  int MSBL = digitalRead(encoderPinL1); //MSB = most significant bit
  int LSBL = digitalRead(encoderPinL2); //LSB = least significant bit

  int encodedL = (MSBL << 1) |LSBL; //converting the 2 pin value to single number
  int sumL  = (lastEncodedL << 2) | encodedL; //adding it to the previous encoded value

  if(sumL == 0b1101 || sumL == 0b0100 || sumL == 0b0010 || sumL == 0b1011) encoderValueL --; //clockwise movement
  if(sumL == 0b1110 || sumL == 0b0111 || sumL == 0b0001 || sumL == 0b1000) encoderValueL ++; //counter-clockwise movement
  lastEncodedL = encodedL;
}
void updateEncoderR(){
  int MSBR = digitalRead(encoderPinR1); //MSB = most significant bit
  int LSBR = digitalRead(encoderPinR2); //LSB = least significant bit

  int encodedR = (MSBR << 1) |LSBR; //converting the 2 pin value to single number
  int sumR  = (lastEncodedR << 2) | encodedR; //adding it to the previous encoded value

  if(sumR == 0b1101 || sumR == 0b0100 || sumR == 0b0010 || sumR == 0b1011) encoderValueR --; //clockwise movement
  if(sumR == 0b1110 || sumR == 0b0111 || sumR == 0b0001 || sumR == 0b1000) encoderValueR ++; //counter-clockwise movement
  lastEncodedR = encodedR;

}
void stop(){
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
  int motorspeed = P*Kp + I*Ki + D*Kd;
  int motorspeedL = 120 + motorspeed; // basespeed = 120
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
void to_goal(){
    int x,y = 0;
    double x2 = 2.4; //tbc
    double y2 = 0.77; //tbc
    int theta2 = 15; //tbc
    int theta = 0;
    int gamma = 3;
    int lamda = 6;
    int h = 1;
    int d = 0.17;
    int r = 43;
    // calculate polar variables
    double rho = sqrt((x2 - x)*(x2-x) + (y2-y)*(y2-y));
    double atan = atan2((y2 - y),(x2 - x));
    double phi = atan - theta2;
    double alpha = phi + theta2 - theta;
    // calculate control laws
    double v = gamma * cos(alpha) * rho;
    double w = lamda * alpha + gamma * cos(alpha) * sin(alpha)*(alpha + h*phi)/alpha;
    // calculate Vr and Vl
    double vr = v + d*w/2;
    double vl = v - d*w/2;
    double wr = vr / r * 60 / (2 * 3.14);
    double wl = vl / r * 60 / (2 * 3.14);
    double PPMR = 4200*wr;
    double PPML = 4200*wl;
    
    float realPWM_R = PPMR * 130 / (6000*60);
    float realPWM_L = PPML * 130 / (6000*60);
    Serial.print("PWM L: ");
    Serial.println(realPWM_L);
    Serial.print("PWM R: ");
    Serial.println(realPWM_R);

    digitalWrite(inR1, 1); 
    digitalWrite(inR2, 0);
    digitalWrite(inL1, 1); 
    digitalWrite(inL2, 0);
    analogWrite(enR, realPWM_R);
    analogWrite(enL, realPWM_L);
}
