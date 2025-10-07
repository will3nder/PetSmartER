#pragma once
#include "ft232h_spi.h"
#include <vector>

// RFM9x: Class for controlling a LoRa radio module
class RFM9x {
public:
     // Constructor: takes reference to SPI interface
     RFM9x(FT232H_SPI& spi);

     // Initializes the radio (reset, LoRa mode)
     void begin();
     // Sets the radio frequency in MHz
     void setFrequency(float mhz);
     // Sends a packet over LoRa
     void sendPacket(const std::vector<uint8_t>& data);
     // Receives a packet from LoRa
     std::vector<uint8_t> receivePacket();

private:
     FT232H_SPI& spi; // Reference to SPI interface
     // Reads a register from the radio
     uint8_t readReg(uint8_t addr);
     // Writes a value to a register
     void writeReg(uint8_t addr, uint8_t value);
};
