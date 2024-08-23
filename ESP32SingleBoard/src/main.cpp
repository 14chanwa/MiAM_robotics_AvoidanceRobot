#include <Arduino.h>
#include <L6470Driver.h>
// #include <I2CMessage.hpp>
// #include <Wire.h>
#include <Parameters.h>
#include <Ps3Controller.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

#include <esp_bt_defs.h>

#include <secret.hpp>

/*
  Pinout:
  - right encoder: A=7, B=6
  - left encoder: A=8, B=9
  - l6470: CS=A2, MOSI=11, MISO=12, SCK=13
  - i2c to esp32: SDA=18, SCL=19
*/

/*
  Steppers
*/

#define CS 5

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

// Encoder pin number: first encoder is in port D, second encoder is in port B.
const unsigned char encoderPinA[2] = {14, 13};
const unsigned char encoderPinB[2] = {27, 12};

// Encoder old B value.
bool oldB[2] = {0, 0};

// Current encoder position.
int32_t encoderCount[2] = {0, 0};

// Interrupt function, called when an encoder interrupt is triggered.
void handleEncoder(unsigned char encoderNumber)
{
  // Get current status.
  bool currentA = digitalRead(encoderPinA[encoderNumber]);

  // The direction of the encoder is given by currentA xor oldB
  encoderCount[encoderNumber] += (oldB[encoderNumber] ^ currentA ? 1 : -1);
  oldB[encoderNumber] = digitalRead(encoderPinB[encoderNumber]);
}

void handleRightEncoder()
{
  handleEncoder(RIGHT_ENCODER_INDEX);
}

void handleLeftEncoder()
{
  handleEncoder(LEFT_ENCODER_INDEX);
}

// /*
//   Communication and state
// */

// using namespace i2c_message;

// I2CSlaveState slaveState = I2CSlaveState::WAITING_FOR_CONFIGURATION;

// #define BUFFER_LENGTH 100
// uint8_t* read_buffer;
// uint8_t* write_buffer;

// void dataRcv(int numBytes)
// {
//   Serial.println("dataRcv");
//   // Receive data
//   uint32_t index = 0;
//   while (index < BUFFER_LENGTH && Wire.available())
//   {
//     read_buffer[index++] = Wire.read();
//   }

//   // Check message
//   bool messageIsCorrect = false;
//   if (index > 0)
//   {
//     if (read_buffer[0] == I2CMessageType::MASTER_CONFIGURATION)
//     {
//       messageIsCorrect = check_message<I2CMasterConfigurationMessage >(read_buffer, index);
//     }
//     else if (read_buffer[0] == I2CMessageType::MASTER_MOTOR_SPEED_TARGET)
//     {
//       messageIsCorrect = check_message<I2CMasterMotorSpeedTargetMessage >(read_buffer, index);
//     }
//   }

//   // Perform actions
//   if (messageIsCorrect)
//   {
//     // if received a configuration message, then re-perform config
//     if (read_buffer[0] == I2CMessageType::MASTER_CONFIGURATION)
//     {
//       Serial.println(">> Received MASTER_CONFIGURATION");
//       I2CMasterConfigurationMessage message = I2CMasterConfigurationMessage(read_buffer);
//       // isStepperInit_ = stepperMotors_->init(message.maxSpeed_, message.maxAcceleration_, MOTOR_KVAL_HOLD,
//       //                                           MOTOR_BEMF[0], MOTOR_BEMF[1], MOTOR_BEMF[2], MOTOR_BEMF[3]);
//       isStepperInit_ = false;

//       // if config failed, then wait for an other attempt
//       if (isStepperInit_)
//       {
//         stepperMotors_->setStepMode(message.stepMode_);
//         Serial.println("Stepper configuration OK");
//         slaveState = I2CSlaveState::NORMAL_OPERATION;
//       }
//       else
//       {
//         Serial.println("Stepper configuration failed");
//         slaveState = I2CSlaveState::WAITING_FOR_CONFIGURATION;
//       }
//     }
//     // if received a motor speed target and normal operations, then apply target
//     else if (read_buffer[0] == I2CMessageType::MASTER_MOTOR_SPEED_TARGET)
//     {
//       Serial.println(">> Received MASTER_MOTOR_SPEED_TARGET");
//       if (slaveState  == I2CSlaveState::NORMAL_OPERATION)
//       {
//         I2CMasterMotorSpeedTargetMessage message = I2CMasterMotorSpeedTargetMessage(read_buffer);
//         std::vector<float> motorSpeed_({0, 0});
//         motorSpeed_[RIGHT_ENCODER_INDEX] = message.rightMotorSpeed_;
//         motorSpeed_[LEFT_ENCODER_INDEX] = message.leftMotorSpeed_;
//         stepperMotors_->setSpeed(motorSpeed_);
//       }
//       else
//       {
//         Serial.println("Failed to apply target: motor not init");
//       }
//     }
//   }
// }

// void dataRqst()
// {
//   Serial.println("dataRqst");
//   // Send a slave information message
//   I2CSlaveInformationMessage message(slaveState, encoderCount[RIGHT_ENCODER_INDEX], encoderCount[LEFT_ENCODER_INDEX]);
//   Serial.print("slaveState: ");
//   Serial.print((uint8_t)message.slaveState_);
//   Serial.println();
//   uint32_t len = message.serialize(write_buffer);
//   Serial.print("Size: ");
//   Serial.println(len);
//   Wire.write(write_buffer, len);
//   Serial.println(">> Sent SLAVE_INFORMATION");
// }

/*
  Tasks
*/

void task_print_encoders(void* parameters)
{
  bool tock = false; 
  for(;;)
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
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void task_handle_ota(void* parameters)
{
  for(;;)
  {
    ArduinoOTA.handle();
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

std::vector<float > targetController({0, 0});

void notify()
{
    // Serial.print(Ps3.data.sensor.accelerometer.x);
    // Serial.print(" ");
    // Serial.print(Ps3.data.sensor.accelerometer.y);
    // Serial.print(" ");
    // Serial.print(Ps3.data.sensor.accelerometer.z);

    // Serial.print(Ps3.data.button.cross);
    // Serial.print(" ");
    // Serial.print(Ps3.data.analog.stick.lx);
    // Serial.print(" ");
    // Serial.print(Ps3.data.analog.stick.ly);

    targetController[0] = -Ps3.data.analog.stick.ly;
    targetController[1] = -Ps3.data.analog.stick.ly;

    /* Uncomment the following if you also want
       to display the gryoscope data: */
    // Serial.print(" ");
    // Serial.print(Ps3.data.sensor.gyroscope.z);

    // Serial.println();
}


/*
  Functions
*/

void stopPs3BT()
{
  Ps3.end();
}


// Compute max stepper motor speed.
int maxSpeed = robotdimensions::maxWheelSpeed / robotdimensions::wheelRadius / robotdimensions::stepSize;
int maxAcceleration = robotdimensions::maxWheelAcceleration / robotdimensions::wheelRadius / robotdimensions::stepSize;

void setup()
{

  Serial.begin(115200);

  pinMode(encoderPinA[0], INPUT);
  pinMode(encoderPinB[0], INPUT);
  pinMode(encoderPinA[1], INPUT);
  pinMode(encoderPinB[1], INPUT);

  attachInterrupt(encoderPinA[RIGHT_ENCODER_INDEX], handleRightEncoder, CHANGE);
  attachInterrupt(encoderPinB[RIGHT_ENCODER_INDEX], handleRightEncoder, CHANGE);
  attachInterrupt(encoderPinA[LEFT_ENCODER_INDEX], handleLeftEncoder, CHANGE);
  attachInterrupt(encoderPinB[LEFT_ENCODER_INDEX], handleLeftEncoder, CHANGE);

  Serial.println("Creating new objects");

  spiWrapper_ = new SPIWrapper(18, 19, 23, 5);
  stepperMotors_ = new miam::L6470(spiWrapper_, 2);

  // read_buffer = new uint8_t[BUFFER_LENGTH];
  // write_buffer = new uint8_t[BUFFER_LENGTH];

  // Wire.begin(MIAM_I2C_SLAVE_ADDRESS);
  // Wire.onReceive(dataRcv);
  // Wire.onRequest(dataRqst);


  // Motor config values.
  const int MOTOR_KVAL_HOLD = 0x30;
  const int MOTOR_BEMF[4] = {0x3B, 0x1430, 0x22, 0x53};

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(SECRET_SSID, SECRET_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.macAddress());

  ArduinoOTA.onStart(stopPs3BT);
  ArduinoOTA.begin();  // Starts OTA
  
  xTaskCreate(task_handle_ota, "task_handle_ota", 10000, NULL, 70, NULL);

  // BT PS3 controller
  Ps3.attach(notify);

  // esp_bd_addr_t addr;
  // const char* macaddress = "14:10:20:30:4:7";
  // int res = sscanf(macaddress, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &addr[0], &addr[1], &addr[2], &addr[3], &addr[4], &addr[5]);
  // Serial.print("macaddress ");
  // Serial.print(addr[0]);
  // Serial.print(", ");
  // Serial.print(addr[1]);
  // Serial.print(", ");
  // Serial.print(addr[2]);
  // Serial.print(", ");
  // Serial.print(addr[3]);
  // Serial.print(", ");
  // Serial.print(addr[4]);
  // Serial.print(", ");
  // Serial.print(addr[5]);
  // Serial.println();
  // Serial.println(res);

  bool res = Ps3.begin();
  Serial.print("Ps3.begin(): ");
  Serial.println(res);

  String address = Ps3.getAddress();
  Serial.println(address);

  // // Print encoder
  // xTaskCreate(task_print_encoders, "task_print_encoders", 1000, NULL, 10, NULL);

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
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }

  stepperMotors_->setStepMode(miam::MICRO_4);


}

std::vector<float> motorSpeed_({0, 0});
#define DEAD_ZONE 10


void loop()
{


  // // put your main code here, to run repeatedly:
  // // Test motors.
  // motorSpeed_[0] = 150;
  // motorSpeed_[1] = 150;
  // stepperMotors_->setSpeed(motorSpeed_);
  // vTaskDelay(3000 / portTICK_PERIOD_MS);
  // motorSpeed_[0] = 0;
  // motorSpeed_[1] = 0;
  // stepperMotors_->setSpeed(motorSpeed_);
  // vTaskDelay(3000 / portTICK_PERIOD_MS);
  // motorSpeed_[0] = -150;
  // motorSpeed_[1] = -150; 
  // stepperMotors_->setSpeed(motorSpeed_);
  // vTaskDelay(3000 / portTICK_PERIOD_MS);
  // motorSpeed_[0] = 0;
  // motorSpeed_[1] = 0;
  // stepperMotors_->setSpeed(motorSpeed_);
  // vTaskDelay(3000 / portTICK_PERIOD_MS);

  if (abs(targetController[0]) > DEAD_ZONE)
  {
    motorSpeed_[0] = maxSpeed * targetController[0] / 128.0;
    motorSpeed_[1] = maxSpeed * targetController[0] / 128.0;

  }
  else
  {
      motorSpeed_[0] = 0;
  motorSpeed_[1] = 0;
  }

  stepperMotors_->setSpeed(motorSpeed_);
  
  vTaskDelay(10 / portTICK_PERIOD_MS);

}

// extern "C" void app_main() {
  
//   initArduino();
//   setup();

//   while(true)
//   {
//     loop();
//   }

// }
