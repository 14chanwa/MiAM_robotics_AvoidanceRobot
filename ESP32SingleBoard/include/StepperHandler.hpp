#ifndef _STEPPER_HANDLER_H
#define _STEPPER_HANDLER_H

#include <L6470Driver.h>

namespace stepper_handler
{
    bool is_inited();
    bool init(int maxSpeed, int maxAcceleration, miam::L6470_STEP_MODE stepMode);

    void setSpeed(float rightValue, float leftValue);
}

#endif