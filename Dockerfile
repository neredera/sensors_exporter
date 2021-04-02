FROM python:3-slim

# RUN pip3 install prometheus_client gpsd-py3 Adafruit-BMP hid pyserial
# spidev (used by Adafruit-BMP for air pressure) does not work for this base image
RUN pip3 install prometheus_client gpsd-py3 hid pyserial

ADD exporter.py /usr/local/bin/sensors_exporter

EXPOSE 9999/tcp

ENTRYPOINT [ "/usr/local/bin/sensors_exporter" ]
