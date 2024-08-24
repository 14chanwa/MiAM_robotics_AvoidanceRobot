#ifndef _ENCODER_HANDLER_H
#define _ENCODER_HANDLER_H

#include <Arduino.h>

#define LEFT_ENCODER_INDEX 0
#define RIGHT_ENCODER_INDEX 1

namespace encoder_handler
{
    void init();
    int32_t getRightValue();
    int32_t getLeftValue();
};

#endif