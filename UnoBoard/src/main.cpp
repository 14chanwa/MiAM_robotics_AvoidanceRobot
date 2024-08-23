#include <Arduino.h>
#include <L6470Driver.h>
#include <I2CMessage.hpp>
#include <Wire.h>
#include <Parameters.h>

/*
  Pinout:
  - right encoder: A=7, B=6
  - left encoder: A=8, B=9
  - l6470: CS=10, MOSI=11, MISO=12, SCK=13
  - i2c to esp32: SDA=18, SCL=19
*/

/*
  Steppers
*/

#define CS 10

SPIWrapper *spiWrapper_;
miam::L6470 *stepperMotors_;
bool isStepperInit_ = false;

// Motor config values.
const int MOTOR_KVAL_HOLD = 0x30;
const int MOTOR_BEMF[4] = {0x3B, 0x1430, 0x22, 0x53};

/*
  Encoders
*/

#define LEFT_ENCODER_INDEX 0
#define RIGHT_ENCODER_INDEX 1

// Read a single bit off a register: return 0 if low, >0 if high.
#define READ_BIT(REG, BIT) (REG & (1 << BIT))

#define ENCODER_TO_REGISTER(ENCODER) (ENCODER ? PINB : PIND)

// Encoder pin number: first encoder is in port D, second encoder is in port B.
const unsigned char encoderPinA[2] = {7, 0};
const unsigned char encoderPinB[2] = {6, 1};

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
  oldB[encoderNumber] = READ_BIT(ENCODER_TO_REGISTER(encoderNumber), encoderPinB[encoderNumber]);
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

/*
  Communication and state
*/

using namespace i2c_message;

I2CSlaveState slaveState = I2CSlaveState::WAITING_FOR_CONFIGURATION;

#define BUFFER_LENGTH 100
uint8_t* read_buffer;
uint8_t* write_buffer;

void dataRcv(int numBytes)
{
  Serial.println("dataRcv");
  // Receive data
  uint32_t index = 0;
  while (index < BUFFER_LENGTH && Wire.available())
  {
    read_buffer[index++] = Wire.read();
  }

  // Check message
  bool messageIsCorrect = false;
  if (index > 0)
  {
    if (read_buffer[0] == I2CMessageType::MASTER_CONFIGURATION)
    {
      messageIsCorrect = check_message<I2CMasterConfigurationMessage >(read_buffer, index);
    }
    else if (read_buffer[0] == I2CMessageType::MASTER_MOTOR_SPEED_TARGET)
    {
      messageIsCorrect = check_message<I2CMasterMotorSpeedTargetMessage >(read_buffer, index);
    }
  }

  // Perform actions
  if (messageIsCorrect)
  {
    // if received a configuration message, then re-perform config
    if (read_buffer[0] == I2CMessageType::MASTER_CONFIGURATION)
    {
      Serial.println(">> Received MASTER_CONFIGURATION");
      I2CMasterConfigurationMessage message = I2CMasterConfigurationMessage(read_buffer);
      // isStepperInit_ = stepperMotors_->init(message.maxSpeed_, message.maxAcceleration_, MOTOR_KVAL_HOLD,
      //                                           MOTOR_BEMF[0], MOTOR_BEMF[1], MOTOR_BEMF[2], MOTOR_BEMF[3]);
      isStepperInit_ = false;

      // if config failed, then wait for an other attempt
      if (isStepperInit_)
      {
        stepperMotors_->setStepMode(message.stepMode_);
        Serial.println("Stepper configuration OK");
        slaveState = I2CSlaveState::NORMAL_OPERATION;
      }
      else
      {
        Serial.println("Stepper configuration failed");
        slaveState = I2CSlaveState::WAITING_FOR_CONFIGURATION;
      }
    }
    // if received a motor speed target and normal operations, then apply target
    else if (read_buffer[0] == I2CMessageType::MASTER_MOTOR_SPEED_TARGET)
    {
      Serial.println(">> Received MASTER_MOTOR_SPEED_TARGET");
      if (slaveState  == I2CSlaveState::NORMAL_OPERATION)
      {
        I2CMasterMotorSpeedTargetMessage message = I2CMasterMotorSpeedTargetMessage(read_buffer);
        std::vector<float> motorSpeed_({0, 0});
        motorSpeed_[RIGHT_ENCODER_INDEX] = message.rightMotorSpeed_;
        motorSpeed_[LEFT_ENCODER_INDEX] = message.leftMotorSpeed_;
        stepperMotors_->setSpeed(motorSpeed_);
      }
      else
      {
        Serial.println("Failed to apply target: motor not init");
      }
    }
  }
}

void dataRqst()
{
  Serial.println("dataRqst");
  // Send a slave information message
  I2CSlaveInformationMessage message(slaveState, encoderCount[RIGHT_ENCODER_INDEX], encoderCount[LEFT_ENCODER_INDEX]);
  Serial.print("slaveState: ");
  Serial.print((uint8_t)message.slaveState_);
  Serial.println();
  uint32_t len = message.serialize(write_buffer);
  Serial.print("Size: ");
  Serial.println(len);
  Wire.write(write_buffer, len);
  Serial.println(">> Sent SLAVE_INFORMATION");
}


/*
  Functions
*/

void setup()
{

  Serial.begin(115200);

  // // Set encoder pins as input.
  // DDRD &= ~(1 << encoderPinA[0]);
  // DDRD &= ~(1 << encoderPinB[0]);
  // DDRB &= ~(1 << encoderPinA[1]);
  // DDRB &= ~(1 << encoderPinB[1]);
  // // Setup interrupt for port B and D.
  // PCMSK2 = 0x00;
  // PCMSK2 |= 1 << encoderPinA[0];
  // PCMSK2 |= 1 << encoderPinB[0];
  // PCMSK0 = 0x00;
  // PCMSK0 |= 1 << encoderPinA[1];
  // PCMSK0 |= 1 << encoderPinB[1];
  // // Enalbe interrupt for port B and D.
  // PCICR = 0b101;

  Serial.println("Creating new objects");

  spiWrapper_ = new SPIWrapper(13, 12, 11, A2);
  stepperMotors_ = new miam::L6470(spiWrapper_, 2);

  // read_buffer = new uint8_t[BUFFER_LENGTH];
  // write_buffer = new uint8_t[BUFFER_LENGTH];

  // Wire.begin(MIAM_I2C_SLAVE_ADDRESS);
  // Wire.onReceive(dataRcv);
  // Wire.onRequest(dataRqst);

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

  stepperMotors_->setStepMode(miam::MICRO_128);
}

long last_tick = 0;
bool tock = false;

void loop()
{
  if (millis() - last_tick > 2000)
  {
    if (tock)
    {
      Serial.print("tock ");
    }
    else
    {
      Serial.print("tick ");
    }
    Serial.print(isStepperInit_);
    Serial.print(", ");
    Serial.print(encoderCount[RIGHT_ENCODER_INDEX]);
    Serial.print(", ");
    Serial.print(encoderCount[LEFT_ENCODER_INDEX]);
    Serial.println();
    tock = !tock;
    last_tick = millis(); 
  }
}
