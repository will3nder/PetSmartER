#include "rfm9x.h"
#include <thread>
#include <chrono>

// Implementation of RFM9x LoRa radio class

// Constructor: stores reference to SPI interface
RFM9x::RFM9x(FT232H_SPI& s) : spi(s) {}

// Initializes the radio: performs hardware reset and sets LoRa mode
void RFM9x::begin() {
    spi.setReset(false); // Assert reset
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    spi.setReset(true); // Deassert reset
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    writeReg(0x01, 0x80); // Set LoRa sleep mode
    writeReg(0x01, 0x81); // Set standby mode
}

// Sets the operating frequency of the radio
void RFM9x::setFrequency(float mhz) {
    uint32_t frf = (uint32_t)((mhz * 1000000.0) / 61.03515625); // Calculate frequency register value
    writeReg(0x06, (frf >> 16) & 0xFF); // MSB
    writeReg(0x07, (frf >> 8) & 0xFF);  // MID
    writeReg(0x08, frf & 0xFF);         // LSB
}

// Sends a packet over LoRa
void RFM9x::sendPacket(const std::vector<uint8_t>& data) {
    writeReg(0x0D, 0); // Set FIFO pointer
    for (auto b : data) writeReg(0x00, b); // Write data to FIFO
    writeReg(0x22, data.size()); // Set payload length
    writeReg(0x01, 0x83); // Set transmit mode
    // Wait for transmission to complete
    while ((readReg(0x12) & 0x08) == 0) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    writeReg(0x12, 0x08); // Clear TxDone flag
}

// Receives a packet from LoRa
std::vector<uint8_t> RFM9x::receivePacket() {
    writeReg(0x01, 0x85); // Set receive mode
    if ((readReg(0x12) & 0x40) == 0) return {}; // No packet received
    uint8_t len = readReg(0x13); // Get packet length
    std::vector<uint8_t> packet;
    writeReg(0x0D, 0); // Set FIFO pointer
    for (int i = 0; i < len; i++)
        packet.push_back(readReg(0x00)); // Read packet data
    writeReg(0x12, 0x40); // Clear RxDone flag
    return packet;
}

// Reads a register from the radio
uint8_t RFM9x::readReg(uint8_t addr) {
    return spi.transfer({static_cast<uint8_t>(addr & 0x7F), static_cast<uint8_t>(0x00)})[1]; // Read register
}

// Writes a value to a register
void RFM9x::writeReg(uint8_t addr, uint8_t value) {
    spi.transfer({uint8_t(addr | 0x80), value}); // Write register
}
