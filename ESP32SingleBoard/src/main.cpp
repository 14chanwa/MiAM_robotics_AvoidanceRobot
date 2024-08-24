#include <Arduino.h>
#include <Parameters.h>
#include <Ps3Controller.h>

#include <WiFiHandler.hpp>
#include <EncoderHandler.hpp>
#include <StepperHandler.hpp>

#include <esp_bt_defs.h>
#include <vector>

#include <secret.hpp>

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
    Serial.print(stepper_handler::is_inited());
    Serial.print(", ");
    Serial.print(encoder_handler::getRightValue());
    Serial.print(", ");
    Serial.print(encoder_handler::getLeftValue());
    Serial.println();
    tock = !tock;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

std::vector<float > targetController({0, 0});

void notify()
{
    targetController[0] = -Ps3.data.analog.stick.ly;
    targetController[1] = -Ps3.data.analog.stick.ly;
}


/*
  Functions
*/

void stopPs3BT()
{
  Ps3.end();
  // stepperMotors_->hardStop();
}


// Compute max stepper motor speed.
int maxSpeed = robotdimensions::maxWheelSpeed / robotdimensions::wheelRadius / robotdimensions::stepSize;
int maxAcceleration = robotdimensions::maxWheelAcceleration / robotdimensions::wheelRadius / robotdimensions::stepSize;

void setup()
{

  Serial.begin(115200);

  // Encoders
  encoder_handler::init();

  // WiFi
  const uint8_t newMacAddress[] = SECRET_AVOIDANCE_ROBOT_WIFI_MAC;
  wifi_handler::setWiFiMACAddress(newMacAddress);
  wifi_handler::initWiFi(SECRET_SSID, SECRET_PASSWORD);
  wifi_handler::setOTAOnStart(stopPs3BT);

  // BT PS3 controller
  Ps3.attach(notify);

  bool res = Ps3.begin(SECRET_BT_MAC);
  Serial.print("Ps3.begin(): ");
  Serial.println(res);

  String address = Ps3.getAddress();
  Serial.println(address);

  // Print encoder
  xTaskCreate(task_print_encoders, "task_print_encoders", 1000, NULL, 10, NULL);

  // Steppers
  while (!stepper_handler::is_inited())
  {
    if (!stepper_handler::init(maxSpeed, maxAcceleration, miam::MICRO_4))
    {
      Serial.print(".");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
}

std::vector<float> motorSpeed_({0, 0});
#define DEAD_ZONE 10

void loop()
{

  if (abs(targetController[0]) > DEAD_ZONE)
  {
    motorSpeed_[RIGHT_ENCODER_INDEX] = maxSpeed * targetController[0] / 128.0;
    motorSpeed_[LEFT_ENCODER_INDEX] = maxSpeed * targetController[0] / 128.0;

  }
  else
  {
      motorSpeed_[RIGHT_ENCODER_INDEX] = 0;
      motorSpeed_[LEFT_ENCODER_INDEX] = 0;
  }

  stepper_handler::setSpeed(motorSpeed_[RIGHT_ENCODER_INDEX], motorSpeed_[LEFT_ENCODER_INDEX]);
  
  vTaskDelay(10 / portTICK_PERIOD_MS);

}
