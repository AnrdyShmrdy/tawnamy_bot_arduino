#include <SimplyAtomic.h>

#define LENCA 14 // Left-Yellow
#define LENCB 15 // Left-Green
#define RENCA 16 // Right-Yellow
#define RENCB 17 // Right-Green

volatile int posiL = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
volatile int posiR = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/

void setup() {
  Serial.begin(9600);
  pinMode(LENCA,INPUT);
  pinMode(LENCB,INPUT);
  pinMode(RENCA,INPUT);
  pinMode(RENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(LENCA),readEncoderL,RISING);
  attachInterrupt(digitalPinToInterrupt(RENCA),readEncoderR,RISING);
}

void loop() {
  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int posL = 0; 
  int posR = 0; 

  ATOMIC() {
    posL = posiL;
    posR = posiR;
  }

  Serial.print(posL);
  Serial.print(" ");
  Serial.println(posR);
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