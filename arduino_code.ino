#include <AccelStepper.h>
#include <Wire.h>

const int dirPin = 2; //direction Pin
const int stepPin = 3; //pulse Pin
const int enPin = 4; //enable Pin

// PID constants (students can edit these to adjust accuracy)
float kp = 1; //*1
float kd = 0.025; //*2
float ki = 0.5; //*3
int output = 0; //output from PID algorithm

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin); //create instance of stepper

long prevT = 0; //previous time
float errorPrev = 0; //previous error

// Define maximum and minimum integral limits
const float MAX_INTEGRAL = 100.0; //maximum integral limit
const float MIN_INTEGRAL = -100.0; //minimum integral limit

float integral = 0; //integral term

void setup() {
  Serial.begin(9600);
  stepper.disableOutputs(); //disable outputs initially
  stepper.setMaxSpeed(10000);
  stepper.setCurrentPosition(0); //zero current stepper position
  stepper.enableOutputs(); //enable outputs for motor
}

void loop() {
  PID();
}

void PID() {

  float target = 90.0 * 1023.0 / 270; // *4 This is the target step we wish to achieve converting from degrees to POT ticks.
  target = constrain(target, 0, 1023); //contrains target to the min/max POT ticks
  Serial.print("target "); //prints out the target
  Serial.print(target); 
  Serial.print(",");

  // Find time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 ); //determine change in time
  prevT = currT; //reset current time

  
  Serial.print("potenValue "); //print out the POT value
  Serial.print(analogRead(A0)); 
  Serial.print(",");

  // PID calculation
  int error = target - analogRead(A0);
  integral = integral + error*deltaT;
  float derivative = (error - errorPrev)/(deltaT);
  errorPrev = error;
 
  // Clamp the integral term
  if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
  if (integral < MIN_INTEGRAL) integral = MIN_INTEGRAL;

  // Control signal
  float output = kp*error + kd*derivative + ki*integral; //PID output (In POT ticks)
  Serial.print("output "); //prints output
  Serial.print(output); 
  Serial.print(",");

  float stepperTarget = round(((((output * 270) / 1023) * 3200) / 360)); //converts POT ticks to steps as motor is set to 3200 steps/revolution
  stepperTarget = (constrain(stepperTarget, -2400, 2400)); //clamps error down too maximum movable steps based on POT
  Serial.print("stepperTarget "); //prints motor's target steps
  Serial.println(stepperTarget);
  stepper.move(stepperTarget);

  // Moves motors for a certain time before repeating PID calculations
  long currT2 = millis();
  while (analogRead(A0) < 1023 && analogRead(A0) > 0 && (millis() - currT2) < 50) { // *5 the period of motor movement can be adjusted
      stepper.setSpeed(2* stepperTarget); //*6 sets motor speed
      stepper.runSpeed(); //steps the motor
 }
}



const int SDA_Pin = 20;
const int SCL_Pin = 21;
const int ledPin = 13; // *** Change LED Pin here

int LED_Byte = 0;
 
void setup() {
  // Arduino joins I2C bus as slave with address 8
  Wire.begin(0x8);
  
  // Call receiveEvent function when data received                
  Wire.onReceive(receiveEvent);
  
  // Setup pin 13 as output and turn LED off at the beginning
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Turn off 20k-50k ohm built-in pull up resistors at pins specified
  digitalWrite(SDA_Pin, LOW);
  digitalWrite(SCL_Pin, LOW);
}
 
// Function that executes whenever data is received from master device, the Pi 5
void receiveEvent(int howMany) {
  LED_Byte = Wire.read(); // receive byte as an integer
  digitalWrite(ledPin, LED_Byte); // turn on/off LED based on byte information
  Serial.println(LED_Byte);
  
}

void loop() {
  delay(100); // Keep waiting for data
}

