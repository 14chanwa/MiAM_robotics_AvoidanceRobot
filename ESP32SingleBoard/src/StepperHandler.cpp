#include <StepperHandler.hpp>
#include <Arduino.h>
#include <EncoderHandler.hpp>

namespace stepper_handler
{
    // Pinout
    const uint8_t sck = 21;
    const uint8_t miso = 19;
    const uint8_t mosi = 18;
    const uint8_t cs = 5;

    SPIWrapper spiWrapper_(sck, miso, mosi, cs);
    miam::L6470 stepperMotors_(&spiWrapper_, 2);
    bool isStepperInit_ = false;

    std::vector<float> motorSpeed_({0, 0});

    // Motor config values.
    const int MOTOR_KVAL_HOLD = 0x30;
    const int MOTOR_BEMF[4] = {0x3B, 0x1430, 0x22, 0x53};

    bool is_inited()
    {
        return isStepperInit_;
    }

    bool init(int maxSpeed, int maxAcceleration, miam::L6470_STEP_MODE stepMode)
    {
        isStepperInit_ = stepperMotors_.init(maxSpeed, maxAcceleration, MOTOR_KVAL_HOLD,
                                             MOTOR_BEMF[0], MOTOR_BEMF[1], MOTOR_BEMF[2], MOTOR_BEMF[3]);
        if (isStepperInit_)
        {
            stepperMotors_.setStepMode(stepMode);
            log_i("Stepper init success");
        }
        else
        {
            log_i("Stepper init failed");
        }
        return isStepperInit_;
    }

    void setSpeed(float rightValue, float leftValue)
    {
        motorSpeed_[RIGHT_ENCODER_INDEX] = rightValue;
        motorSpeed_[LEFT_ENCODER_INDEX] = leftValue;
        stepperMotors_.setSpeed(motorSpeed_);
    }
}