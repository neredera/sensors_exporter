#!/bin/bash

# go to current directory
cd "${0%/*}"

# install needed packages
sudo apt-get install python3-smbus i2c-tools git python3-venv python3-pip libhidapi-libusb0

# activate a virtual environment
python3 -m venv .

# install python modules
# this module supports BMP085 and BMP180 but is deprecated.
# the replacement only supports BMP280 and not BMP085 and BMP180.
python3 -m pip install prometheus_client gpsd-py3 Adafruit-BMP hid


# for zgmco2 (access to USB device)
sudo cp 92-zgmco2.rules /etc/udev/rules.d/92-zgmco2.rules
sudo groupadd -f zgmco2
sudo usermod -a -G zgmco2 pi
# relogin is needed that the new group becomes active

chmod +x exporter.py

# sudo systemctl daemon-reload

sudo systemctl enable $(pwd)/sensors_exporter.service

sudo systemctl start sensors_exporter.service

python3 exporter.py --help

sudo systemctl status sensors_exporter.service

#python3 exporter.py --use_gps True
#python3 exporter.py --use_bmp085 True --use_gps True
