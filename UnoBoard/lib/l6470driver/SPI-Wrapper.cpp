/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include <SPI-Wrapper.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include <iostream>
#include <cstring>

#include <SPI.h>

#define SPI_PACKET_DELAY_MICROSECONDS 5

SPIWrapper::SPIWrapper(uint8_t chipSelectPin):
    chipSelectPin_(chipSelectPin),
    frequency_(4000000)
{
    pinMode(chipSelectPin_, OUTPUT);
    digitalWrite(chipSelectPin_, HIGH);
    SPI.begin();
}


int SPIWrapper::spiReadWriteSingle(uint8_t* data, uint8_t const& len)
{
    static struct spi_ioc_transfer spiCtrl;

    // First element: send address and wait.
    spiCtrl.tx_buf = &data[0];
    spiCtrl.rx_buf = &data[0];
    spiCtrl.len = len;

    return spiReadWrite(1, &spiCtrl);
}

int SPIWrapper::spiReadWrite(uint8_t const& numberOfPackets, struct spi_ioc_transfer* spiCtrl)
{
    SPISettings settings = SPISettings(frequency_, MSBFIRST, SPI_MODE0);

    SPI.beginTransaction(settings);

    for (int i=0; i<numberOfPackets; i++)
    {
        digitalWrite(chipSelectPin_, LOW);
        for (uint32_t j=0; j<spiCtrl[i].len; j++)
        {
            spiCtrl[i].rx_buf[j] = SPI.transfer(spiCtrl[i].tx_buf[j]);
        }
        digitalWrite(chipSelectPin_, HIGH);
        delayMicroseconds(SPI_PACKET_DELAY_MICROSECONDS);
    }
    SPI.endTransaction();

    int res = 0;

    return res;
}

