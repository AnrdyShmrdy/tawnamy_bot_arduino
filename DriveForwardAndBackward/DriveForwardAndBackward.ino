#define enA 4
#define in1 5
#define in2 6
#define in3 7
#define in4 8
#define enB 9 
//14 Works for PWM and Digital
//15 Works for Digital and PWM
//ENA and ENB cannot be 21, 20, 19, or 18



void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); //This Works
  pinMode(in3, OUTPUT); //This Works
  pinMode(in4, OUTPUT); //This Works
  // Set initial rotation direction
  //Values >= 65 will work, but above 65 is recommended
  analogWrite(enA, 90);
  analogWrite(enB, 90); //this works
}

void loop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); //this works
  digitalWrite(in4, HIGH); //this works
  delay(2000);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); //this works
  digitalWrite(in4, LOW); //this works
  delay(2000);
}