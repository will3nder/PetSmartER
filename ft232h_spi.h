#pragma once
#include <libftdi1/ftdi.h>
#include <vector>
#include <stdexcept>

// FT232H_SPI: Class for SPI communication using FTDI FT232H chip
class FT232H_SPI {
public:
     // Constructor: Initializes FTDI context and GPIOs
     FT232H_SPI();
     // Destructor: Cleans up FTDI context
     ~FT232H_SPI();

     // Initializes FT232H for SPI
     void begin();
     // Set Chip Select (CS) pin
     void setCS(bool high);
     // Set Reset pin
     void setReset(bool high);
     // Set Enable (EN) pin
     void setEN(bool high);
     // Set DIO0 pin
     void setDIO0(bool high);

     // Transfer data over SPI and receive response
     std::vector<uint8_t> transfer(const std::vector<uint8_t>& data);

private:
     struct ftdi_context* ftdi; // FTDI context pointer

     // GPIO values and directions
     unsigned char gpioLowVal;
     unsigned char gpioLowDir;
     unsigned char gpioHighVal;
     unsigned char gpioHighDir;

     // Configures MPSSE mode (not implemented)
     void configureMPSSE();
     // Writes GPIO values to FT232H
     void writeGPIO();
};
