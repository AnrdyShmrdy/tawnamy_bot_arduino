//An issue with this is that both motors get out of sync with each other when running this code
//Why this is I'm not sure...could be hardware or software in nature
//Either way I want to fix this issue at some point
#include <SimplyAtomic.h>

#define LENCA 14 // Left-Yellow (Left motor, rear-facing & upright)
#define LENCB 15 // Left-Green (Left motor, rear-facing & upright)
#define RENCA 16 // Right-Yellow (Right motor, rear-facing & upright)
#define RENCB 17 // Right-Green (Right motor, rear-facing & upright)
//White wire = Output1 on L298N controller
//Red wire = Output2 on L298N controoler
//Reversing the red+white wires will cause this code to not work
//If you want to reverse them, you'll have to change some value from positive to negative in this code somewhere
//I don't know where though...yet
#define PWMA 4 //Orange (L298N)
#define IN1 5 //Yellow (L298N)
#define IN2 6 //Green (L298N)
#define IN3 7 //Blue (L298N)
#define IN4 8 //Purple (L298N)
#define PWMB 9 //White (L298N)

// specify variables as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
volatile int posiL = 0;
volatile int posiR = 0;

long prevT = 0;
float eprevL = 0;
float eintegralL = 0;
float eprevR = 0;
float eintegralR = 0;

void setup() {
  Serial.begin(9600);
  pinMode(LENCA,INPUT);
  pinMode(LENCB,INPUT);
  pinMode(RENCA,INPUT);
  pinMode(RENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(LENCA),readEncoderL,RISING);
  attachInterrupt(digitalPinToInterrupt(RENCA),readEncoderR,RISING);
  pinMode(PWMA,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(PWMB,OUTPUT);
  
  Serial.println("target pos");
}

void loop() {

  // set target position
  //int target = 1200;
  int target = 250*sin(prevT/1e6);

  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int posL = 0;
  int posR = 0;
  ATOMIC() {
    posL = posiL;
    posR = posiR;
  }
  
  // error
  int errorL = posL - target;
  int errorR = posR - target;

  // derivative
  float dedtL = (errorL-eprevL)/(deltaT);
  float dedtR = (errorR-eprevR)/(deltaT);

  // integral
  eintegralL = eintegralL + errorL*deltaT;
  eintegralR = eintegralR + errorR*deltaT;

  // control signal
  float uL = kp*errorL + kd*dedtL + ki*eintegralL;
  float uR = kp*errorR + kd*dedtR + ki*eintegralR;

  // motor power
  float pwrL = fabs(uL);
  if( pwrL > 55 ){
    pwrL = 55;
  }
  float pwrR = fabs(uR);
  if( pwrR > 55 ){
    pwrR = 55;
  }

  // motor direction
  int dirL = 1;
  if(uL<0){
    dirL = -1;
  }
  int dirR = 1;
  if(uR<0){
    dirR = -1;
  }

  // signal the motor
  setMotor(dirL,pwrL,PWMA,IN1,IN2);
  setMotor(dirR,pwrR,PWMB,IN3,IN4);

  // store previous error
  eprevL = errorL;
  eprevR = errorR;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(posL);
  Serial.print(" ");
  Serial.print(posR);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readEncoderL(){
  int b = digitalRead(LENCB);
  if(b > 0){
    posiL++;
  }
  else{
    posiL--;
  }
}

void readEncoderR(){
  int b = digitalRead(RENCB);
  if(b > 0){
    posiR++;
  }
  else{
    posiR--;
  }
}