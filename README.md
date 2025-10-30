# PetSmartER

A project for ECEN 1400

## Components:
* 2x Rasberry Pi PICO 2W
* 2x High Torque Servos
* Generic Water Sensor
* XIAO RA4M1
* 2x Adafruit RFM95W LoRa Radio
* Adafruit Ultimate GPS Breakout - PA1616S
* General Ubuntu/Debian Server
* Adafruit FT232H Breakout

## Modules:
* A dog collar with GPS trakcing and LoRa communication
* An automatic doggie door
* An automatic water refiller
* A base which hosts a web server for sensor information and communicates via WiFi and LoRa

## Team Members:
* William Ender
* Jackson Lastowski
* Kaden Ritzer
* Asher Levin

## Dependencies and Building:

### Server (server.py & lora-gps.service)

#### Download Dependencies with:

`sudo apt update`\
`sudo apt install -y
  python3 python3-venv python3-pip
  build-essential
  libusb-1.0-0-dev libftdi1-dev`

`pip install pyftdi Adafruit-Blinka adafruti-circuitpython-rfm9x Flask` 

#### Create a udev rool so non-root can access the device:
Create `/etc/udev/rules.d/11-ftdi.rules` with:

`SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6001", GROUP="plugdev", MODE="0666"`\
`SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6011", GROUP="plugdev", MODE="0666"`\
`SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6010", GROUP="plugdev", MODE="0666"`\
`SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6014", GRUP="plugdev", MODE="0666"`
Then:

`sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG plugdev,dialout $USER
`

#### Edit and deploy the systemd service
Edit the lora-gps.service file to replace all instances of "yourusername" with YOUR user name.

Then:
`sudo systemctl daemon-reload`\
`sudo systemctl enable --now lora-gps.service`\
`sudo journalctl -u lora-gps -f`


### Dog Collar (xiao.ino)

#### Required Arduino Libraries
* Arduino
* SPI
* Adafruit GPS Library
* LoRa

##### Arduino CLI commands:
`arduino-cli lib install "Adafruit GPS Library"`
`arduino-cli lib install "LoRa"`

#### Compilation
Use either the arduino-cli or the Arduino IDE\
For the Arduino IDE open the .ino, select the board (XIAO RA4M1) and serial port, then click Upload.\

For arduinocli:
`arduino-cli compile --fqbn seeed:samd:xiao_ra4m1 /path/to/xiao`\
`arduino-cli upload --fqbn seeed:samd:xiao_ra4m1 -p /dev/ttyUSB0 /path/to/xiao`