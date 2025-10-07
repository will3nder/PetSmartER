#include "ft232h_spi.h"
#include "rfm9x.h"
#include <iostream>
#include <thread>

// Entry point for LoRa communication demo
int main() {
    try {
        // Create SPI interface and initialize
        FT232H_SPI spi;
        spi.begin();
        spi.setEN(true); // Enable LoRa module

        // Create RFM9x radio object and initialize
        RFM9x radio(spi);
        radio.begin();
        radio.setFrequency(915.0); // Set frequency to 915 MHz

        std::cout << "Listening for packets..." << std::endl;

        // Main loop: receive and echo packets
        while (true) {
            auto pkt = radio.receivePacket(); // Try to receive a packet
            if (!pkt.empty()) {
                std::string text(pkt.begin(), pkt.end()); // Convert packet to string
                std::cout << "Received: " << text << std::endl;
                radio.sendPacket(pkt); // Echo back received packet
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Small delay
        }

    } catch (const std::exception& e) {
        // Print error message if exception occurs
        std::cerr << "ERROR: " << e.what() << std::endl;
    }
}
