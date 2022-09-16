#include <PrintStream.h>

int E1 = 7;
int M1 = 6;
int M1_pwm = 0;
int E2 = 13;
int M2 = 12;
int M2_pwm = 0;
String a = "";
String M1_pwm_str;
String M2_pwm_str;

// Motor encoder output pulses per 360 degree revolution (measured manually)
#define ENC_COUNT_REV 12
#define GEAR_RATIO 75
 
// Encoder output to Arduino Interrupt pin. Tracks the pulse count.
#define ENC_IN_RIGHT_A 3
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_RIGHT_B 4

// Encoder output to Arduino Interrupt pin. Tracks the pulse count.
#define ENC_IN_LEFT_A 2
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 8
 
// True = Forward; False = Reverse
boolean Direction_right = true;
 
// Keep track of the number of right wheel pulses
volatile long right_wheel_pulse_count = 0;

// True = Forward; False = Reverse
boolean Direction_left = true;
 
// Keep track of the number of right wheel pulses
volatile long left_wheel_pulse_count = 0;

// One-second interval for measurements
int interval = 1000;

// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;
 
// Variable for RPM measuerment
float rpm_right = 0;
float rpm_left = 0;

void setup()
{
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  Serial.begin(9600);

  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_pulse, RISING);
}

void loop()
{

  while(Serial.available()) {
    Serial.read();
    a= Serial.readString();// read the incoming data as string
    //Serial.println(a);
  }
  if (a != ""){
    M1_pwm_str = a.substring(0,4);
    //Serial.println(M1_pwm_str);
    M2_pwm_str = a.substring(5);
    //Serial.println(M2_pwm_str);
    M1_pwm = M1_pwm_str.toInt();
    M2_pwm = M2_pwm_str.toInt();
  }
  if (M2_pwm<0){
    digitalWrite(M2, LOW);
  }
  else{
    digitalWrite(M2,HIGH);
    M2_pwm = map(abs(M2_pwm),0, 255,255,0);
  }
  if (M1_pwm<0){
    digitalWrite(M1, LOW);
  }
  else{
    digitalWrite(M1,HIGH);
    M1_pwm = map(abs(M1_pwm),0, 255,255,0);
  }
  analogWrite(E1, abs(M1_pwm));   //PWM Speed Control
  analogWrite(E2, abs(M2_pwm));
  //Serial.println(M1_pwm);
  delay(30);

   // Record the time
  currentMillis = millis();
 
  // If one second has passed, print the number of pulses
  if (currentMillis - previousMillis > interval) {
 
    previousMillis = currentMillis;
 
    // Calculate revolutions per minute
    //Serial.println(right_wheel_pulse_count);
    rpm_right = (float)(right_wheel_pulse_count * 60 / (ENC_COUNT_REV * GEAR_RATIO));
    rpm_left = (float)(left_wheel_pulse_count * 60 / (ENC_COUNT_REV * GEAR_RATIO));
    //Serial.println(rpm_right);
    //Serial.println(rpm_left);
    right_wheel_pulse_count = 0;
    left_wheel_pulse_count = 0;
    Serial << rpm_right << ":" << rpm_left << endl;
  }
}

void right_wheel_pulse() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
    right_wheel_pulse_count++;
  }
  else {
    right_wheel_pulse_count--;
  }
}

void left_wheel_pulse() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if(val == LOW) {
    Direction_left = false; // Reverse
  }
  else {
    Direction_left = true; // Forward
  }
   
  if (Direction_left) {
    left_wheel_pulse_count++;
  }
  else {
    left_wheel_pulse_count--;
  }
}
