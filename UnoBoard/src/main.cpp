#include <Arduino.h>
#include <L6470Driver.h> 
#include <Parameters.h>

#define CS A2

SPIWrapper* spiWrapper_;
miam::L6470* stepperMotors_;
bool isStepperInit_ = false;

void setup() {

  Serial.begin(115200);

  Serial.println("Creating new objects");

  spiWrapper_ = new SPIWrapper(CS);
  stepperMotors_ = new miam::L6470(spiWrapper_, 2);
  
  Serial.println("Configure steppers");

  // Motor config values.
  const int MOTOR_KVAL_HOLD = 0x30;
  const int MOTOR_BEMF[4] = {0x3B, 0x1430, 0x22, 0x53};

  // Compute max stepper motor speed.
  int maxSpeed = robotdimensions::maxWheelSpeed / robotdimensions::wheelRadius / robotdimensions::stepSize;
  int maxAcceleration = robotdimensions::maxWheelAcceleration / robotdimensions::wheelRadius / robotdimensions::stepSize;
  
  // Initialize both motors.
  while (!isStepperInit_)
  {
    isStepperInit_ = stepperMotors_->init(maxSpeed, maxAcceleration, MOTOR_KVAL_HOLD,
                                             MOTOR_BEMF[0], MOTOR_BEMF[1], MOTOR_BEMF[2], MOTOR_BEMF[3]);

    if (isStepperInit_)
    {
      Serial.println("Stepper init success");
    }
    else
    {
      Serial.print(".");
      delay(1000);
    }
  }
}

std::vector<float> motorSpeed_({0, 0});

void loop() {
  // put your main code here, to run repeatedly:
  // Test motors.
  motorSpeed_[0] = 150;
  motorSpeed_[1] = 150;
  stepperMotors_->setSpeed(motorSpeed_);
  delay(3000);
  motorSpeed_[0] = 0;
  motorSpeed_[1] = 0;
  stepperMotors_->setSpeed(motorSpeed_);
  delay(3000);
  motorSpeed_[0] = -150;
  motorSpeed_[1] = -150; 
  stepperMotors_->setSpeed(motorSpeed_);
  delay(3000);
  motorSpeed_[0] = 0;
  motorSpeed_[1] = 0;
  stepperMotors_->setSpeed(motorSpeed_);
  delay(3000);
}
