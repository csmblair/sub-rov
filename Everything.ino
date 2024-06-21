//***********************DC MOTORS***********************
#define dc_enA 9
#define dc_in1 4
#define dc_in2 5
#define dc_enB 10
#define dc_in3 6
#define dc_in4 7

int dc_motorSpeedA = 0;
int dc_motorSpeedB = 0;


//***********************STEPPER MOTOR***********************
// include Arduino stepper motor library
#include <Stepper.h>

// define number of steps per revolution
#define STEPS 200

// define stepper motor control pins
#define step_IN1  13
#define step_IN2  12
#define step_IN3  11
#define step_IN4  2

// initialize stepper library
Stepper stepper(STEPS, step_IN4, step_IN2, step_IN3, step_IN1);

// joystick pot output is connected to Arduino A0
#define step_joystick  A5

void setup()
{
//***********************DC MOTORS***********************
  pinMode(dc_enA, OUTPUT);
  pinMode(dc_enB, OUTPUT);
  pinMode(dc_in1, OUTPUT);
  pinMode(dc_in2, OUTPUT);
  pinMode(dc_in3, OUTPUT);
  pinMode(dc_in4, OUTPUT);
}

void loop()
{
//***********************DC MOTORS***********************
  int dc_xAxis = analogRead(A0); // Read Joysticks X-axis
  int dc_yAxis = analogRead(A1); // Read Joysticks Y-axis

  // Y-axis used for forward and backward control
  if (dc_yAxis < 470) {
    // Set Motor A backward
    digitalWrite(dc_in1, HIGH);
    digitalWrite(dc_in2, LOW);
    // Set Motor B backward
    digitalWrite(dc_in3, HIGH);
    digitalWrite(dc_in4, LOW);
    // Convert the declining Y-axis readings for going backward from 470 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
    dc_motorSpeedA = map(dc_yAxis, 470, 0, 0, 255);
    dc_motorSpeedB = map(dc_yAxis, 470, 0, 0, 255);
  }
  else if (dc_yAxis > 550) {
    // Set Motor A forward
    digitalWrite(dc_in1, LOW);
    digitalWrite(dc_in2, HIGH);
    // Set Motor B forward
    digitalWrite(dc_in3, LOW);
    digitalWrite(dc_in4, HIGH);
    // Convert the increasing Y-axis readings for going forward from 550 to 1023 into 0 to 255 value for the PWM signal for increasing the motor speed
    dc_motorSpeedA = map(dc_yAxis, 550, 1023, 0, 255);
    dc_motorSpeedB = map(dc_yAxis, 550, 1023, 0, 255);
  }
  // If joystick stays in middle the motors are not moving
  else {
    dc_motorSpeedA = 0;
    dc_motorSpeedB = 0;
  }

  // X-axis used for left and right control
  if (dc_xAxis < 470) {
    // Convert the declining X-axis readings from 470 to 0 into increasing 0 to 255 value
    int dc_xMapped = map(dc_xAxis, 470, 0, 0, 255);
    // Move to left - decrease left motor speed, increase right motor speed
    dc_motorSpeedA = dc_motorSpeedA - dc_xMapped;
    dc_motorSpeedB = dc_motorSpeedB + dc_xMapped;
    // Confine the range from 0 to 255
    if (dc_motorSpeedA < 0) {
      dc_motorSpeedA = 0;
    }
    if (dc_motorSpeedB > 255) {
      dc_motorSpeedB = 255;
    }
  }
  if (dc_xAxis > 550) {
    // Convert the increasing X-axis readings from 550 to 1023 into 0 to 255 value
    int dc_xMapped = map(dc_xAxis, 550, 1023, 0, 255);
    // Move right - decrease right motor speed, increase left motor speed
    dc_motorSpeedA = dc_motorSpeedA + dc_xMapped;
    dc_motorSpeedB = dc_motorSpeedB - dc_xMapped;
    // Confine the range from 0 to 255
    if (dc_motorSpeedA > 255) {
      dc_motorSpeedA = 255;
    }
    if (dc_motorSpeedB < 0) {
      dc_motorSpeedB = 0;
    }
  }
  // Prevent buzzing at low speeds (Adjust according to your motors. My motors couldn't start moving if PWM value was below value of 70)
  if (dc_motorSpeedA < 70) {
    dc_motorSpeedA = 0;
  }
  if (dc_motorSpeedB < 70) {
    dc_motorSpeedB = 0;
  }
  analogWrite(dc_enA, dc_motorSpeedA); // Send PWM signal to motor A
  analogWrite(dc_enB, dc_motorSpeedB); // Send PWM signal to motor B


//***********************STEPPER MOTOR***********************
  // read analog value from the potentiometer
  int step_val = analogRead(step_joystick);

  // if the joystic is in the middle ===> stop the motor
  if(  (step_val > 500) && (step_val < 523) )
  {
    digitalWrite(step_IN1, LOW);
    digitalWrite(step_IN2, LOW);
    digitalWrite(step_IN3, LOW);
    digitalWrite(step_IN4, LOW);
  }

  else
  {
    // move the motor in the first direction
    while (step_val >= 523)
    {
      // map the speed between 5 and 500 rpm
      int step_speed_ = map(step_val, 523, 1023, 5, 500);
      // set motor speed
      stepper.setSpeed(step_speed_);

      // move the motor (1 step)
      stepper.step(1);

      step_val = analogRead(step_joystick);
    }

    // move the motor in the other direction
    while (step_val <= 500)
    {
      // map the speed between 5 and 500 rpm
      int step_speed_ = map(step_val, 500, 0, 5, 500);
      // set motor speed
      stepper.setSpeed(step_speed_);

      // move the motor (1 step)
      stepper.step(-1);

      step_val = analogRead(step_joystick);
    }

  }

}
