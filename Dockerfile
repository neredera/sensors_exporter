FROM python:3-slim

COPY install-packages.sh .
RUN ./install-packages.sh

RUN pip3 install prometheus_client gpsd-py3 Adafruit-BMP hid pyserial

ADD exporter.py /usr/local/bin/sensors_exporter

EXPOSE 9999/tcp

ENTRYPOINT [ "/usr/local/bin/sensors_exporter" ]
