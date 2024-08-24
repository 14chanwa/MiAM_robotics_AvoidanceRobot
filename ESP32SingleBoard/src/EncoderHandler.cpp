#include <EncoderHandler.hpp>

namespace encoder_handler
{

    // Encoder pin number
    const unsigned char encoderPinA[2] = {34, 32};
    const unsigned char encoderPinB[2] = {35, 33};

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

    void init()
    {
        pinMode(encoderPinA[0], INPUT);
        pinMode(encoderPinB[0], INPUT);
        pinMode(encoderPinA[1], INPUT);
        pinMode(encoderPinB[1], INPUT);

        attachInterrupt(encoderPinA[RIGHT_ENCODER_INDEX], handleRightEncoder, CHANGE);
        attachInterrupt(encoderPinB[RIGHT_ENCODER_INDEX], handleRightEncoder, CHANGE);
        attachInterrupt(encoderPinA[LEFT_ENCODER_INDEX], handleLeftEncoder, CHANGE);
        attachInterrupt(encoderPinB[LEFT_ENCODER_INDEX], handleLeftEncoder, CHANGE);

        log_i("Encoder inited");
    }

    int32_t getRightValue()
    {
        return encoderCount[RIGHT_ENCODER_INDEX];
    }

    int32_t getLeftValue()
    {
        return encoderCount[LEFT_ENCODER_INDEX];
    }
};