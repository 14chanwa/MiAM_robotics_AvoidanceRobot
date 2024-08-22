#include <Arduino.h>
#include <L6470Driver.h> 
#include <Parameters.h>
#include <I2CMessage.hpp>

/*
  Pinout:
  - right encoder: A=7, B=6
  - left encoder: A=8, B=9
  - l6470: CS=A2, MOSI=11, MISO=12, SCK=13
  - i2c to esp32: SDA=18, SCL=19 
*/

#define CS A2

SPIWrapper* spiWrapper_;
miam::L6470* stepperMotors_;
bool isStepperInit_ = false;

#define LEFT_ENCODER_INDEX 0
#define RIGHT_ENCODER_INDEX 1

// Read a single bit off a register: return 0 if low, >0 if high.
#define READ_BIT(REG, BIT) (REG & (1 << BIT))

#define ENCODER_TO_REGISTER(ENCODER) (ENCODER ? PINB : PIND)

// Encoder pin number: first encoder is in port D, second encoder is in port B.
const unsigned char encoderPinA[2] = {7,0};
const unsigned char encoderPinB[2] = {6,1};

// Encoder old B value.
bool oldB[2] = {0, 0};

// Current encoder position.
int32_t encoderCount[2] = {0, 0};

// Interrupt function, called when an encoder interrupt is triggered.
void handleEncoder(unsigned char encoderNumber)
{
  // Get current status.
  bool currentA = READ_BIT(ENCODER_TO_REGISTER(encoderNumber), encoderPinA[encoderNumber]);

  // The direction of the encoder is given by currentA xor oldB
  encoderCount[encoderNumber] += (oldB[encoderNumber] ^ currentA ? 1 : -1);
  oldB[encoderNumber] =  READ_BIT(ENCODER_TO_REGISTER(encoderNumber), encoderPinB[encoderNumber]);
}

// Interrupt for first encoder.
ISR(PCINT2_vect)
{
  handleEncoder(0);
}

// Interrupt for second encoder.
ISR(PCINT0_vect)
{
  handleEncoder(1);
}

void setup() {

  Serial.begin(115200);

  // Set encoder pins as input.
  DDRD &= ~(1 << encoderPinA[0]);
  DDRD &= ~(1 << encoderPinB[0]);
  DDRB &= ~(1 << encoderPinA[1]);
  DDRB &= ~(1 << encoderPinB[1]);
  // Setup interrupt for port B and D.
  PCMSK2 = 0x00;
  PCMSK2 |= 1 << encoderPinA[0];
  PCMSK2 |= 1 << encoderPinB[0];
  PCMSK0 = 0x00;
  PCMSK0 |= 1 << encoderPinA[1];
  PCMSK0 |= 1 << encoderPinB[1];
  // Enalbe interrupt for port B and D.
  PCICR = 0b101;

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
  
  // // Initialize both motors.
  // while (!isStepperInit_)
  // {
  //   isStepperInit_ = stepperMotors_->init(maxSpeed, maxAcceleration, MOTOR_KVAL_HOLD,
  //                                            MOTOR_BEMF[0], MOTOR_BEMF[1], MOTOR_BEMF[2], MOTOR_BEMF[3]);

  //   if (isStepperInit_)
  //   {
  //     Serial.println("Stepper init success");
  //   }
  //   else
  //   {
  //     Serial.print(".");
  //     delay(1000);
  //   }
  // }

  // stepperMotors_->setStepMode(miam::MICRO_128);

}

std::vector<float> motorSpeed_({0, 0});

void loop() {
  // // put your main code here, to run repeatedly:
  // // Test motors.
  // motorSpeed_[0] = 150;
  // motorSpeed_[1] = 150;
  // stepperMotors_->setSpeed(motorSpeed_);
  // delay(3000);
  // motorSpeed_[0] = 0;
  // motorSpeed_[1] = 0;
  // stepperMotors_->setSpeed(motorSpeed_);
  // delay(3000);
  // motorSpeed_[0] = -150;
  // motorSpeed_[1] = -150; 
  // stepperMotors_->setSpeed(motorSpeed_);
  // delay(3000);
  // motorSpeed_[0] = 0;
  // motorSpeed_[1] = 0;
  // stepperMotors_->setSpeed(motorSpeed_);
  // delay(3000);

  Serial.print("encoderValues: ");
  Serial.print(encoderCount[0]);
  Serial.print(", ");
  Serial.println(encoderCount[1]);
  delay(100);
}
