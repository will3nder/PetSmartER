# RFM9x Listener

This project interfaces a Linux system with the Adafruit FT232H USB-to-GPIO board, which then communicates with the Adafruit RFM9x LoRa radio breakout. The FT232H acts as a USB-to-SPI bridge, allowing your Linux machine to send and receive data to the RFM9x module.

## Dependencies

Install the required packages:

```sh
sudo apt install libftdi1-dev build-essential
```

## Compilation

Compile the project using:

```sh
g++ main.cpp ft232h_spi.cpp rfm9x.cpp -o rfm9x_listener -lftdi1 -lpthread -std=c++17
```

## FT232H Kernel Driver

To prevent the kernel serial driver from claiming the FT232H device, add the following rule to `/etc/udev/rules.d/99-ft232h-unbind.rules`:

```sh
SUBSYSTEM=="usb", DRIVER=="ftdi_sio", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", \
RUN+="/bin/sh -c 'echo $kernel > /sys/bus/usb/drivers/ftdi_sio/unbind'"
```

This ensures the FT232H is available for direct access via libftdi.

## Hardware Connections

- Connect the FT232H board to your Linux machine via USB.
- Wire the FT232H SPI pins to the RFM9x breakout according to the Adafruit documentation.

## References

- [Adafruit FT232H Guide](https://learn.adafruit.com/adafruit-ft232h-breakout)
- [Adafruit RFM9x LoRa Radio Guide](https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/pinouts)
- [FTDI Chip LibMPSSE-SPI Examples](https://ftdichip.com/software-examples/mpsse-projects/libmpsse-spi-examples/)
- [libFTDI Open Source Library](https://www.intra2net.com/en/developer/libftdi/)
