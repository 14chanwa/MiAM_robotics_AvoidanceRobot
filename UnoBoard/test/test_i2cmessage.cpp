#include "unity.h"
#include <I2CMessage.hpp>

uint8_t *write_buffer;
#define BUFFER_LENGTH 100

using namespace i2c_message;

void setUp(void)
{
    // set stuff up here
    write_buffer = new uint8_t[BUFFER_LENGTH];
}

void tearDown(void)
{
    // clean stuff up here
}

void test_I2CMasterConfigurationMessage(void)
{
    I2CMasterConfigurationMessage message = I2CMasterConfigurationMessage(500, 1000);

    Serial.print("Initial I2CMasterConfigurationMessage: maxSpeed_=");
    Serial.print(message.maxSpeed_);
    Serial.print(", maxAcceleration_=");
    Serial.println(message.maxAcceleration_);

    uint32_t len = message.serialize(write_buffer);
    Serial.print("Serialized I2CMasterConfigurationMessage length ");
    Serial.println(len);

    I2CMasterConfigurationMessage message2 = I2CMasterConfigurationMessage(write_buffer);
    Serial.print("Read I2CMasterConfigurationMessage: maxSpeed_=");
    Serial.print(message2.maxSpeed_);
    Serial.print(", maxAcceleration_=");
    Serial.println(message2.maxAcceleration_);

    uint16_t computed_crc;
    memcpy(&computed_crc, &(write_buffer[len-sizeof(computed_crc)]), sizeof(computed_crc));
    TEST_ASSERT_EQUAL_INT16(_compute_CRC(write_buffer, len-2), computed_crc);

    TEST_ASSERT_EQUAL_INT32(message.maxSpeed_, message2.maxSpeed_);
    TEST_ASSERT_EQUAL_INT32(message.maxAcceleration_, message2.maxAcceleration_);
}

void test_I2CMasterMotorSpeedTargetMessage(void)
{
    I2CMasterMotorSpeedTargetMessage message = I2CMasterMotorSpeedTargetMessage(500, 1000);

    Serial.print("Initial I2CMasterMotorSpeedTargetMessage: rightMotorSpeed_=");
    Serial.print(message.rightMotorSpeed_);
    Serial.print(", leftMotorSpeed_=");
    Serial.println(message.leftMotorSpeed_);

    uint32_t len = message.serialize(write_buffer);
    Serial.print("Serialized I2CMasterMotorSpeedTargetMessage length ");
    Serial.println(len);

    I2CMasterMotorSpeedTargetMessage message2 = I2CMasterMotorSpeedTargetMessage(write_buffer);
    Serial.print("Read I2CMasterMotorSpeedTargetMessage: rightMotorSpeed_=");
    Serial.print(message2.rightMotorSpeed_);
    Serial.print(", leftMotorSpeed_=");
    Serial.println(message2.leftMotorSpeed_);

    uint16_t computed_crc;
    memcpy(&computed_crc, &(write_buffer[len-sizeof(computed_crc)]), sizeof(computed_crc));
    TEST_ASSERT_EQUAL_INT16(_compute_CRC(write_buffer, len-2), computed_crc);
    
    TEST_ASSERT_EQUAL_FLOAT(message.rightMotorSpeed_, message2.rightMotorSpeed_);
    TEST_ASSERT_EQUAL_FLOAT(message.leftMotorSpeed_, message2.leftMotorSpeed_);
}


void test_I2CSlaveInformationMessage(void)
{
    I2CSlaveInformationMessage message = I2CSlaveInformationMessage(I2CSlaveState::NORMAL_OPERATION, 500, 1000);

    Serial.print("Initial I2CSlaveInformationMessage: slaveState_=");
    Serial.print(message.slaveState_);
    Serial.print(", rightEncoderValue_=");
    Serial.print(message.rightEncoderValue_);
    Serial.print(", leftEncoderValue_=");
    Serial.println(message.leftEncoderValue_);

    uint32_t len = message.serialize(write_buffer);
    Serial.print("Serialized I2CSlaveInformationMessage length ");
    Serial.println(len);

    I2CSlaveInformationMessage message2 = I2CSlaveInformationMessage(write_buffer);
    Serial.print("Read I2CSlaveInformationMessage: slaveState_=");
    Serial.print(message2.slaveState_);
    Serial.print(", rightEncoderValue_=");
    Serial.print(message2.rightEncoderValue_);
    Serial.print(", leftEncoderValue_=");
    Serial.println(message2.leftEncoderValue_);

    uint16_t computed_crc;
    memcpy(&computed_crc, &(write_buffer[len-sizeof(computed_crc)]), sizeof(computed_crc));
    TEST_ASSERT_EQUAL_INT16(_compute_CRC(write_buffer, len-2), computed_crc);
    
    TEST_ASSERT_EQUAL_CHAR(message.slaveState_, message2.slaveState_);
    TEST_ASSERT_EQUAL_INT32(message.rightEncoderValue_, message2.rightEncoderValue_);
    TEST_ASSERT_EQUAL_INT32(message.leftEncoderValue_, message2.leftEncoderValue_);
}

int runUnityTests(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_I2CMasterConfigurationMessage);
    RUN_TEST(test_I2CMasterMotorSpeedTargetMessage);
    RUN_TEST(test_I2CSlaveInformationMessage);
    return UNITY_END();
}

/**
 * For Arduino framework
 */
void setup()
{
    // Wait ~2 seconds before the Unity test runner
    // establishes connection with a board Serial interface
    delay(2000);

    runUnityTests();
}
void loop() {}
