/*
 * Author: Automatic Addison
 * Website: https://automaticaddison.com
 * Description: Calculate the angular velocity in radians/second of a DC motor
 * with a built-in encoder (forward = positive; reverse = negative) 
 */

int RPWM_Output = 9; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
int LPWM_Output = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)

// Motor B connections
//const int enB = 10;
//const int in3 = 7;
//const int in4 = 8;
int RPWM_Output2 = 11; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
int LPWM_Output2 = 10; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)

 
// Motor encoder output pulses per 360 degree revolution (measured manually)
#define ENC_COUNT_REV 2950
 
// Encoder output to Arduino Interrupt pin. Tracks the pulse count.
#define ENC_IN_RIGHT_A 2
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_RIGHT_B 4


#define ENC_IN_LEFT_A 3
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 5
 
// True = Forward; False = Reverse
boolean Direction_right = true;

// True = Forward; False = Reverse
boolean Direction_left = true;
 
 
// Keep track of the number of right wheel pulses
volatile long right_wheel_pulse_count = 0;

 volatile long left_wheel_pulse_count = 0;

// One-second interval for measurements
int interval = 1000;
  
// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;
 
// Variable for RPM measuerment
float rpm_right = 0;

// Variable for RPM measuerment
float rpm_left = 0;
 
 
// Variable for angular velocity measurement
float ang_velocity_right = 0;
float ang_velocity_right_deg = 0;

// Variable for angular velocity measurement
float ang_velocity_left = 0;
float ang_velocity_left_deg = 0;
 
const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.29578;
 
void setup() {
 
  // Open the serial port at 9600 bps
  Serial.begin(9600); 
 
  // Set pin states of the encoder
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);

    pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
 
  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);
 attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_pulse, RISING);
    analogWrite(LPWM_Output, 80);
    analogWrite(RPWM_Output, 0);
    analogWrite(LPWM_Output2, 0);
    analogWrite(RPWM_Output2, 80);
 
   
}
 
void loop() {



  
  // Record the time
  currentMillis = millis();
 
  // If one second has passed, print the number of pulses
  if (currentMillis - previousMillis > interval) {

    Serial.println("_____RIGHT WHEEL____");
 
    previousMillis = currentMillis;
 
    // Calculate revolutions per minute
    rpm_right = (float)(right_wheel_pulse_count * 60 / ENC_COUNT_REV);
    ang_velocity_right = rpm_right * rpm_to_radians;   
    ang_velocity_right_deg = ang_velocity_right * rad_to_deg;
     
    Serial.print(" Pulses: ");
    Serial.println(right_wheel_pulse_count);
    Serial.print(" Speed: ");
    Serial.print(rpm_right);
    Serial.println(" RPM");
    Serial.print(" Angular Velocity: ");
    Serial.print(rpm_right);
    Serial.print(" rad per second");
    Serial.print("\t");
    Serial.print(ang_velocity_right_deg);
    Serial.println(" deg per second");

    Serial.print(ang_velocity_right * 0.06);
    Serial.println("ms^-1");
    Serial.println();
 
    right_wheel_pulse_count = 0;

Serial.println("_____LEFT WHEEL____");

        rpm_left = (float)(left_wheel_pulse_count * 60 / ENC_COUNT_REV);
    ang_velocity_left = rpm_left * rpm_to_radians;   
    ang_velocity_left_deg = ang_velocity_left * rad_to_deg;
     
    Serial.print(" Pulses: ");
    Serial.println(left_wheel_pulse_count);
    Serial.print(" Speed: ");
    Serial.print(rpm_left);
    Serial.println(" RPM");
    Serial.print(" Angular Velocity: ");
    Serial.print(rpm_left);
    Serial.print(" rad per second");
    Serial.print("\t");
    Serial.print(ang_velocity_left_deg);
    Serial.println(" deg per second");

    Serial.print(ang_velocity_left * 0.06);
    Serial.println("ms^-1");
    Serial.println();
 
    left_wheel_pulse_count = 0;




    
   
  }
}
 
// Increment the number of pulses by 1
void right_wheel_pulse() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = true; // Reverse
  }
  else {
    Direction_right = false; // Forward
  }
   
  if (Direction_right) {
    right_wheel_pulse_count++;
  }
  else {
    right_wheel_pulse_count--;
  }
}



// Increment the number of pulses by 1
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
