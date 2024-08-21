/// \file drivers/SPI-Wrapper.h
/// \brief Thread-safe wrapper for SPI communication.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef SPI_WRAPPER
#define SPI_WRAPPER

    #include <string>
    #include <SPI.h>

     struct spi_ioc_transfer {
        uint8_t*       tx_buf;
        uint8_t*       rx_buf;
        uint32_t       len;
    };

    /// \brief Thread-safe wrapper for SPI communication,.
    class SPIWrapper{
        public:
            /// \brief Constructor
            /// \param[in] chipSelectPin
            SPIWrapper(uint8_t chipSelectPin = 10);

            /// \brief Send and receives an array of data over spi.
            /// \param[in] data Data to send - the buffer is overwritten with new data.
            /// \param[in] len Length of the input buffer.
            /// \return <0 on error.
            int spiReadWriteSingle(uint8_t* data, uint8_t const& len);

            int spiReadWrite(uint8_t const& numberOfPackets, struct spi_ioc_transfer* spiCtrl);

        private:
            uint32_t frequency_; ///< Frequency, in Hz.
            uint8_t chipSelectPin_;
    };

#endif
