#ifndef _I2C_MESSAGE_HPP
#define _I2C_MESSAGE_HPP

#include <Arduino.h>
#include <util/crc16.h>

namespace i2c_message
{

    enum I2CSlaveState : uint8_t
    {
        WAITING_FOR_CONFIGURATION = 0,
        NORMAL_OPERATION = 1
    };

    enum I2CMessageType : uint8_t
    {
        SLAVE_INFORMATION = 0,
        MASTER_CONFIGURATION = 1,
        MASTER_MOTOR_SPEED_TARGET = 2
    };

    template <typename T>
    void _serialize(const T &value, uint8_t *buffer, uint32_t &index)
    {
        memcpy(&buffer[index], &value, sizeof(T));
        index += sizeof(T);
    }

    template <typename T>
    T _parse(const uint8_t *buffer, uint32_t &index)
    {
        T res;
        memcpy(&res, &buffer[index], sizeof(T));
        index += sizeof(T);
        return res;
    }

    void _print_buffer(const uint8_t *buffer, const uint32_t len)
    {
        for (uint32_t i = 0; i < len; i++)
        {
            Serial.print(buffer[i]);
            Serial.print(" ");
        }
        Serial.println();
    }

    uint16_t _compute_CRC(const uint8_t *buffer, const uint32_t len)
    {
        uint16_t crc = 0;
        for (uint32_t i = 0; i < len; i++)
        {
            crc = _crc16_update(crc, buffer[i]);
        }
        return crc;
    }

    class I2CMessage
    {
    public:
        I2CMessage(I2CMessageType messageType) : messageType_(messageType) {}
        I2CMessageType get_message_type() { return messageType_; };
        virtual uint32_t serialize(uint8_t *buffer) = 0;

        void add_header(uint8_t *buffer, uint32_t &index)
        {
            _serialize<I2CMessageType>(messageType_, buffer, index);
        };

        const I2CMessageType messageType_;
    };

    class I2CSlaveInformationMessage : public I2CMessage
    {
    public:
        I2CSlaveInformationMessage() : I2CSlaveInformationMessage(I2CSlaveState::NORMAL_OPERATION, 0, 0) {}

        I2CSlaveInformationMessage(I2CSlaveState slaveState, int32_t rightEncoderValue, int32_t leftEncoderValue) : I2CMessage(I2CMessageType::SLAVE_INFORMATION), rightEncoderValue_(rightEncoderValue), leftEncoderValue_(leftEncoderValue) {}

        I2CSlaveInformationMessage(uint8_t *buffer) : I2CSlaveInformationMessage()
        {
            uint32_t len = 0;
            // First byte is messageType
            len++;
            slaveState_ = _parse<I2CSlaveState>(buffer, len);
            rightEncoderValue_ = _parse<uint32_t>(buffer, len);
            leftEncoderValue_ = _parse<uint32_t>(buffer, len);
        }

        uint32_t serialize(uint8_t *buffer)
        {
            uint32_t len = 0;
            I2CMessage::add_header(buffer, len);
            _serialize<I2CSlaveState>(slaveState_, buffer, len);
            _serialize<uint32_t>(rightEncoderValue_, buffer, len);
            _serialize<uint32_t>(leftEncoderValue_, buffer, len);
            _serialize<int16_t>(_compute_CRC(buffer, len), buffer, len);
            _print_buffer(buffer, len);
            return len;
        }

        I2CSlaveState slaveState_;
        int32_t rightEncoderValue_;
        int32_t leftEncoderValue_;
    };

    class I2CMasterConfigurationMessage : public I2CMessage
    {
    public:
        I2CMasterConfigurationMessage() : I2CMasterConfigurationMessage(0, 0) {}

        I2CMasterConfigurationMessage(int32_t maxSpeed, int32_t maxAcceleration) : I2CMessage(I2CMessageType::MASTER_CONFIGURATION), maxSpeed_(maxSpeed), maxAcceleration_(maxAcceleration) {}

        I2CMasterConfigurationMessage(uint8_t *buffer) : I2CMasterConfigurationMessage()
        {
            uint32_t len = 0;
            // First byte is messageType
            len++;
            maxSpeed_ = _parse<int32_t>(buffer, len);
            maxAcceleration_ = _parse<int32_t>(buffer, len);
        }

        uint32_t serialize(uint8_t *buffer)
        {
            uint32_t len = 0;
            I2CMessage::add_header(buffer, len);
            _serialize<int32_t>(maxSpeed_, buffer, len);
            _serialize<int32_t>(maxAcceleration_, buffer, len);
            _serialize<int16_t>(_compute_CRC(buffer, len), buffer, len);
            _print_buffer(buffer, len);
            return len;
        }

        int32_t maxSpeed_;
        int32_t maxAcceleration_;
    };

    class I2CMasterMotorSpeedTargetMessage : public I2CMessage
    {
    public:
        I2CMasterMotorSpeedTargetMessage() : I2CMasterMotorSpeedTargetMessage(0.0, 0.0) {}

        I2CMasterMotorSpeedTargetMessage(float rightMotorSpeed, float leftMotorSpeed) : I2CMessage(I2CMessageType::MASTER_MOTOR_SPEED_TARGET), rightMotorSpeed_(rightMotorSpeed), leftMotorSpeed_(leftMotorSpeed) {}

        I2CMasterMotorSpeedTargetMessage(uint8_t *buffer) : I2CMasterMotorSpeedTargetMessage()
        {
            uint32_t len = 0;
            // First byte is messageType
            len++;
            rightMotorSpeed_ = _parse<float>(buffer, len);
            leftMotorSpeed_ = _parse<float>(buffer, len);
        }

        uint32_t serialize(uint8_t *buffer)
        {
            uint32_t len = 0;
            I2CMessage::add_header(buffer, len);
            _serialize<float>(rightMotorSpeed_, buffer, len);
            _serialize<float>(leftMotorSpeed_, buffer, len);
            _serialize<int16_t>(_compute_CRC(buffer, len), buffer, len);
            _print_buffer(buffer, len);
            return len;
        }

        float rightMotorSpeed_;
        float leftMotorSpeed_;
    };

}

#endif
