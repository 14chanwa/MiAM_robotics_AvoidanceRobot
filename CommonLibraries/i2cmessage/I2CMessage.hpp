#ifndef _I2C_MESSAGE_HPP
#define _I2C_MESSAGE_HPP

#include <Arduino.h>
#include <FastCRC.h>
#include <L6470Driver.h>

// #define DEBUG
#ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
#endif

#define MIAM_I2C_SLAVE_ADDRESS 0x42

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
            DEBUG_PRINT(buffer[i]);
            DEBUG_PRINT(" ");
        }
        DEBUG_PRINTLN();
    }

    uint16_t _compute_CRC(const uint8_t *buffer, const uint32_t len)
    {
        FastCRC16 CRC16;
        return CRC16.mcrf4xx(buffer, len);
    }

    template<typename T>
    uint32_t expected_size()
    {
        return T::expected_header_size() + T::expected_payload_size() + sizeof(uint16_t);
    }

    template <typename T>
    bool check_message(const uint8_t* buffer, const uint32_t& len)
    {
        // Check message length
        if (len != expected_size<T>())
        {
            return false;
        }
        // Check CRC
        uint16_t computed_crc;
        memcpy(&computed_crc, &(buffer[len-sizeof(computed_crc)]), sizeof(uint16_t));
        if (_compute_CRC(buffer, len-sizeof(uint16_t)) != computed_crc)
        {
            return false;
        }
        return true;
    }

    /*
        Class definitions
    */

    class I2CMessage
    {
    public:
        I2CMessage(I2CMessageType messageType) : messageType_(messageType) {}
        I2CMessageType get_message_type() { return messageType_; };
        
        uint32_t serialize(uint8_t *buffer) {
            uint32_t len = 0;
            write_header(buffer, len);
            write_payload(buffer, len);
            write_crc(buffer, len);
            _print_buffer(buffer, len);
            return len;
        }
    
        static uint32_t expected_header_size() { return sizeof(I2CMessageType); }
        static uint32_t expected_payload_size() { return 0; };

    protected:
        void write_header(uint8_t *buffer, uint32_t &index)
        {
            _serialize<I2CMessageType>(messageType_, buffer, index);
        };

        virtual void write_payload(uint8_t *buffer, uint32_t& index) = 0;
        
        void write_crc(uint8_t *buffer, uint32_t &index)
        {
            _serialize<int16_t>(_compute_CRC(buffer, index), buffer, index);
        }
    private:
        const I2CMessageType messageType_;
    };

    class I2CSlaveInformationMessage : public I2CMessage
    {
    public:
        I2CSlaveInformationMessage() : I2CSlaveInformationMessage(I2CSlaveState::NORMAL_OPERATION, 0, 0) {}

        I2CSlaveInformationMessage(I2CSlaveState slaveState, int32_t rightEncoderValue, int32_t leftEncoderValue) : 
            I2CMessage(I2CMessageType::SLAVE_INFORMATION), slaveState_(slaveState), rightEncoderValue_(rightEncoderValue), leftEncoderValue_(leftEncoderValue) {}

        I2CSlaveInformationMessage(uint8_t *buffer) : I2CSlaveInformationMessage()
        {
            uint32_t len = expected_header_size();
            slaveState_ = _parse<I2CSlaveState>(buffer, len);
            rightEncoderValue_ = _parse<uint32_t>(buffer, len);
            leftEncoderValue_ = _parse<uint32_t>(buffer, len);
        }

        static uint32_t expected_payload_size() {
            return sizeof(I2CSlaveState) + sizeof(uint32_t) + sizeof(uint32_t);
        };

        void write_payload(uint8_t *buffer, uint32_t& index) {
            _serialize<I2CSlaveState>(slaveState_, buffer, index);
            _serialize<uint32_t>(rightEncoderValue_, buffer, index);
            _serialize<uint32_t>(leftEncoderValue_, buffer, index);
        };

        I2CSlaveState slaveState_;
        int32_t rightEncoderValue_;
        int32_t leftEncoderValue_;
    };

    class I2CMasterConfigurationMessage : public I2CMessage
    {
    public:
        I2CMasterConfigurationMessage() : I2CMasterConfigurationMessage(0, 0, miam::L6470_STEP_MODE::FULL) {}

        I2CMasterConfigurationMessage(int32_t maxSpeed, int32_t maxAcceleration, miam::L6470_STEP_MODE stepMode) : 
            I2CMessage(I2CMessageType::MASTER_CONFIGURATION), maxSpeed_(maxSpeed), maxAcceleration_(maxAcceleration), stepMode_(stepMode) {}

        I2CMasterConfigurationMessage(uint8_t *buffer) : I2CMasterConfigurationMessage()
        {
            uint32_t len = expected_header_size();
            maxSpeed_ = _parse<int32_t>(buffer, len);
            maxAcceleration_ = _parse<int32_t>(buffer, len);
            stepMode_ = _parse<miam::L6470_STEP_MODE>(buffer, len);
        }

        static uint32_t expected_payload_size() {
            return sizeof(int32_t) + sizeof(uint32_t) + sizeof(miam::L6470_STEP_MODE);
        }

        void write_payload(uint8_t *buffer, uint32_t& index) {
            _serialize<int32_t>(maxSpeed_, buffer, index);
            _serialize<int32_t>(maxAcceleration_, buffer, index);
            _serialize<miam::L6470_STEP_MODE>(stepMode_, buffer, index);
        }

        int32_t maxSpeed_;
        int32_t maxAcceleration_;
        miam::L6470_STEP_MODE stepMode_;
    };

    class I2CMasterMotorSpeedTargetMessage : public I2CMessage
    {
    public:
        I2CMasterMotorSpeedTargetMessage() : I2CMasterMotorSpeedTargetMessage(0.0, 0.0) {}

        I2CMasterMotorSpeedTargetMessage(float rightMotorSpeed, float leftMotorSpeed) : I2CMessage(I2CMessageType::MASTER_MOTOR_SPEED_TARGET), rightMotorSpeed_(rightMotorSpeed), leftMotorSpeed_(leftMotorSpeed) {}

        I2CMasterMotorSpeedTargetMessage(uint8_t *buffer) : I2CMasterMotorSpeedTargetMessage()
        {
            uint32_t len = expected_header_size();
            rightMotorSpeed_ = _parse<float>(buffer, len);
            leftMotorSpeed_ = _parse<float>(buffer, len);
        }


        static uint32_t expected_payload_size() {
            return sizeof(float) + sizeof(float);
        }

        void write_payload(uint8_t *buffer, uint32_t& index) {
            _serialize<float>(rightMotorSpeed_, buffer, index);
            _serialize<float>(leftMotorSpeed_, buffer, index);
        }

        float rightMotorSpeed_;
        float leftMotorSpeed_;
    };

}

#endif
