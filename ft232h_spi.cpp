// Implementation of FT232H_SPI class for SPI communication using FTDI FT232H chip
#include "ft232h_spi.h"
#include <iostream>
#include <unistd.h>
#include <cstring>

// Constructor: Initializes FTDI context and sets default GPIO values
FT232H_SPI::FT232H_SPI() {
    ftdi = ftdi_new(); // Create new FTDI context
    if (!ftdi) throw std::runtime_error("ftdi_new failed");

    gpioLowVal = 0x08; // Default low GPIO value
    gpioLowDir = 0b00111111; // Low GPIO direction (output)
    gpioHighVal = 0x00; // Default high GPIO value
    gpioHighDir = 0x01; // High GPIO direction (output)
}

// Destructor: Cleans up FTDI context
FT232H_SPI::~FT232H_SPI() {
    ftdi_usb_close(ftdi); // Close USB connection
    ftdi_free(ftdi); // Free FTDI context
}

// Initializes the FT232H for SPI communication
void FT232H_SPI::begin() {
    // Open FT232H device
    if (ftdi_usb_open(ftdi, 0x0403, 0x6014) < 0)
        throw std::runtime_error("Cannot open FT232H");

    // Reset FTDI device
    if (ftdi_set_bitmode(ftdi, 0x0, BITMODE_RESET) < 0)
        throw std::runtime_error("Failed to reset FTDI");

    // Enable MPSSE mode for SPI
    if (ftdi_set_bitmode(ftdi, 0x0, BITMODE_MPSSE) < 0)
        throw std::runtime_error("Failed to enable MPSSE");

    ftdi_usb_purge_buffers(ftdi); // Purge buffers

    // Set SPI clock divider
    uint16_t clkdiv = 29; // ~1MHz
    unsigned char cmd[3] = {0x86, static_cast<unsigned char>(clkdiv & 0xFF), static_cast<unsigned char>(clkdiv >> 8)};
    if (ftdi_write_data(ftdi, cmd, 3) != 3)
        throw std::runtime_error("Failed to set SPI clock");

    writeGPIO(); // Initialize GPIOs
}

// Writes current GPIO values and directions to FT232H
void FT232H_SPI::writeGPIO() {
    unsigned char buf[3];

    // Write low GPIO
    buf[0] = 0x80;
    buf[1] = gpioLowVal;
    buf[2] = gpioLowDir;
    if (ftdi_write_data(ftdi, buf, 3) != 3)
        throw std::runtime_error("Failed to write low GPIO");

    // Write high GPIO
    buf[0] = 0x82;
    buf[1] = gpioHighVal;
    buf[2] = gpioHighDir;
    if (ftdi_write_data(ftdi, buf, 3) != 3)
        throw std::runtime_error("Failed to write high GPIO");
}

// Sets the Chip Select (CS) pin high or low
void FT232H_SPI::setCS(bool high) {
    if (high) gpioLowVal |= 0x01; // Set CS high
    else gpioLowVal &= ~0x01; // Set CS low
    writeGPIO();
}

// Sets the Reset pin high or low
void FT232H_SPI::setReset(bool high) {
    if (high) gpioHighVal |= 0x01; // Set Reset high
    else gpioHighVal &= ~0x01; // Set Reset low
    writeGPIO();
}

// Sets the EN (Enable) pin high or low
void FT232H_SPI::setEN(bool high) {
    if (high) gpioLowVal |= 0x20; // Set EN high
    else gpioLowVal &= ~0x20; // Set EN low
    writeGPIO();
}

// Sets the DIO0 pin high or low
void FT232H_SPI::setDIO0(bool high) {
    if (high) gpioLowVal |= 0x10; // Set DIO0 high
    else gpioLowVal &= ~0x10; // Set DIO0 low
    writeGPIO();
}

// Transfers data over SPI and returns received bytes
std::vector<uint8_t> FT232H_SPI::transfer(const std::vector<uint8_t>& data) {
    std::vector<uint8_t> rx(data.size()); // Buffer for received data

    setCS(false); // Assert CS (active)

    for (size_t i = 0; i < data.size(); i++) {
        unsigned char cmd[3] = {0x31, 0x00, 0x00}; // SPI transfer command
        if (ftdi_write_data(ftdi, cmd, 3) != 3)
            throw std::runtime_error("Failed SPI transfer command");

        // Write byte to SPI
        if (ftdi_write_data(ftdi, &data[i], 1) != 1)
            throw std::runtime_error("Failed SPI write byte");

        // Read byte from SPI
        unsigned char read_buf[1];
        int r = ftdi_read_data(ftdi, read_buf, 1);
        if (r != 1) throw std::runtime_error("Failed SPI read byte");
        rx[i] = read_buf[0];
    }

    setCS(true); // Deassert CS
    return rx;
}
