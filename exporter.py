#!/usr/bin/env python3

import argparse
import distutils.util as util
import hid
import serial
import time
import gpsd
import os
import logging
import statistics
import threading

import Adafruit_BMP.BMP085 as BMP085
import prometheus_client

from collections import deque
from prometheus_client.core import (
    InfoMetricFamily, GaugeMetricFamily, CounterMetricFamily)
from threading import Thread

# TODO: Errorhandling, Recovery after errors (e.g. sensor temporary unavailable), error counters
# TODO: Fill zgmco2 from Loop. zgmco2: Do not send data if not available
# TODO: Split in file per sensor
# TODO: Logging for SystemD. More Logging.
# TODO: Own User/Group for Service

PROMETHEUS_NAMESPACE = 'sensor'

class SensorsCollector(object):
    """Collector for sensor data."""

    bmp085 = None
    use_bmp085 = False
    use_gps = False
    zgmco2 = None
    use_zgmco2 = False
    use_gm45 = False
    gm45 = None
    fixedposition = True
    lastaltitudevalues = deque(maxlen = 5760) # Queue for 1 day when a 15sec scrape interval is used
    lastlongitudevalues = deque(maxlen = 5760)
    lastlatitudevalues = deque(maxlen = 5760)

    def __init__(self, use_bmp085, use_gps, use_zgmco2, use_gm45, gm45_device, fixedposition, registry=prometheus_client.REGISTRY):
        self.use_bmp085 = use_bmp085
        self.use_gps = use_gps
        self.use_zgmco2 = use_zgmco2
        self.use_gm45 = use_gm45
        self.fixedposition = fixedposition

        if self.use_bmp085:
            self.bmp085 = BMP085.BMP085(mode=BMP085.BMP085_ULTRAHIGHRES)

        if self.use_gps:
            # Connect to the local gpsd
            gpsd.connect()

        if self.use_zgmco2:
            self.zgmco2 = Zgmco2Sensor()
            Thread(target=self.zgmco2.mainloop).start()

        if self.use_gm45:
            self.gm45 = Gm45Sensor(gm45_device)
            Thread(target=self.gm45.mainloop).start()

        registry.register(self)

    def pressure_to_sealevel_pressure(self, pressure_pa, altitude_m):
        """Calculates the pressure at sealevel when given a known altitude in
        meters. Returns a value in Pascals."""

        p0 = pressure_pa / pow(1.0 - altitude_m/44330.0, 5.255)
        return p0

    def collect(self):
        has_altitude = False
        has_pressure = False

        metrics = []

        if self.use_bmp085:
            bmp085_temperature = GaugeMetricFamily(
                PROMETHEUS_NAMESPACE + '_bmp085_temperature',
                'Temperature of pressure sensor bmp085/bmp180 in °C',
                labels=[])
            metrics.append(bmp085_temperature)
            bmp085_pressure = GaugeMetricFamily(
                PROMETHEUS_NAMESPACE + '_bmp085_pressure',
                'Pressure of pressure sensor bmp085/bmp180 in hPa',
                labels=[])
            metrics.append(bmp085_pressure)

            temperature = self.bmp085.read_temperature()
            has_pressure = True
            pressure = self.bmp085.read_pressure() # in Pascal

            bmp085_temperature.add_metric(labels=[], value=temperature)
            bmp085_pressure.add_metric(labels=[], value=pressure/100)

        if self.use_gps:
            gps_mode = GaugeMetricFamily(
                PROMETHEUS_NAMESPACE + '_gps_mode',
                'Status of GPS reception. 0=No value, 1=No fix, 2=2D fix, 3=3D fix',
                labels=[])
            metrics.append(gps_mode)
            gps_satellites = GaugeMetricFamily(
                PROMETHEUS_NAMESPACE + '_gps_satellites',
                'Number of satellites visible',
                labels=[])
            metrics.append(gps_satellites)
            gps_satellites_used = GaugeMetricFamily(
                PROMETHEUS_NAMESPACE + '_gps_satellites_used',
                'Number of satellites used',
                labels=[])
            metrics.append(gps_satellites_used)
            gps_altitude = GaugeMetricFamily(
                PROMETHEUS_NAMESPACE + '_gps_altitude',
                'Altitude in m',
                labels=[])
            metrics.append(gps_altitude)
            gps_altitude_error = GaugeMetricFamily(
                PROMETHEUS_NAMESPACE + '_gps_altitude_error',
                'Estimated altitude error (95% confidence) in m',
                labels=[])
            metrics.append(gps_altitude_error)
            gps_longitude = GaugeMetricFamily(
                PROMETHEUS_NAMESPACE + '_gps_longitude',
                'Longitude',
                labels=[])
            metrics.append(gps_longitude)
            gps_longitude_error = GaugeMetricFamily(
                PROMETHEUS_NAMESPACE + '_gps_longitude_error',
                'Estimated longitude error (95% confidence) in m',
                labels=[])
            metrics.append(gps_longitude_error)
            gps_latitude = GaugeMetricFamily(
                PROMETHEUS_NAMESPACE + '_gps_latitude',
                'Latitude',
                labels=[])
            metrics.append(gps_latitude)
            gps_latitude_error = GaugeMetricFamily(
                PROMETHEUS_NAMESPACE + '_gps_latitude_error',
                'Estimated latitude error (95% confidence) in m',
                labels=[])
            metrics.append(gps_latitude_error)
            gps_time_error = GaugeMetricFamily(
                PROMETHEUS_NAMESPACE + '_gps_time_error',
                'Estimated time error (95% confidence) in s (not delivered by all receivers)',
                labels=[])
            metrics.append(gps_time_error)
            gps_info = InfoMetricFamily(
                PROMETHEUS_NAMESPACE + '_gps_info',
                'GPS receiver information')
            metrics.append(gps_info)

            # Some infos about the GPS device (driver, port etc.)
            device = gpsd.device()
            gps_info.add_metric(
                labels=[], 
                value={
                    'path': device['path'],
                    'speed': str(device['speed']),
                    'driver': device['driver'],
                })

            # Get gps position
            packet = gpsd.get_current()

            #Indicates the status of the GPS reception, 0=No value, 1=No fix, 2=2D fix, 3=3D fix
            gps_mode.add_metric(labels=[], value=packet.mode)

            gps_satellites.add_metric(labels=[], value=packet.sats)
            gps_satellites_used.add_metric(labels=[], value=packet.sats_valid)
            if packet.mode>=2:
                gps_longitude.add_metric(labels=[], value=packet.lon)
                gps_longitude_error.add_metric(labels=[], value=packet.error['x'])
                if self.fixedposition:
                    self.lastlongitudevalues.append(packet.lon)
                gps_latitude.add_metric(labels=[], value=packet.lat)
                gps_latitude_error.add_metric(labels=[], value=packet.error['y'])
                if self.fixedposition:
                    self.lastlatitudevalues.append(packet.lat)
                gps_time_error.add_metric(labels=[], value=packet.error['t'])

            if packet.mode==3:
                has_altitude = True
                altitude = packet.altitude()
                altitude_for_sealevel = altitude

                gps_altitude.add_metric(labels=[], value=packet.altitude())
                gps_altitude_error.add_metric(labels=[], value=packet.error['v'])
                if self.fixedposition:
                    self.lastaltitudevalues.append(packet.altitude())

        if self.use_zgmco2:
            zgmco2_temperature = GaugeMetricFamily(
                PROMETHEUS_NAMESPACE + '_zgmco2_temperature',
                'Temperature of ZG mini CO2 sensor in °C',
                labels=[])
            metrics.append(zgmco2_temperature)
            zgmco2_co2 = GaugeMetricFamily(
                PROMETHEUS_NAMESPACE + '_zgmco2_co2ppm',
                'CO2 in ppm from ZG mini CO2 sensor',
                labels=[])
            metrics.append(zgmco2_co2)
            zgmco2_info = InfoMetricFamily(
                PROMETHEUS_NAMESPACE + '_zgmco2_info',
                'ZG mini CO2 sensor information')
            metrics.append(zgmco2_info)

            zgmco2_info.add_metric(
                labels=[],
                value={
                    'manufacturer': self.zgmco2.manufacturer,
                    'product': self.zgmco2.product,
                    'serial': self.zgmco2.serial,
                    })

            temperature = self.zgmco2.lasttemperature
            co2 = self.zgmco2.lastco2ppm

            if not temperature is None:
                zgmco2_temperature.add_metric(labels=[], value=temperature)
            if not co2 is None:
                zgmco2_co2.add_metric(labels=[], value=co2)

        if self.use_gm45:
            gm45_info = InfoMetricFamily(
                PROMETHEUS_NAMESPACE + '_gm45_info',
                'GM-45 geiger counter information',
                value={'device': self.gm45.device})
            metrics.append(gm45_info)

            # get a current count
            self.gm45.updateCount()

            gm45_counts = CounterMetricFamily(
                PROMETHEUS_NAMESPACE + '_gm45_counts',
                'GM-45 geiger counter, radioactive events counter',
                value=self.gm45.count)
            metrics.append(gm45_counts)

        if self.use_gps and self.fixedposition:
            calc_median_altitude = GaugeMetricFamily(
                PROMETHEUS_NAMESPACE + '_calc_median_altitude',
                'Median altitude in m over the last day',
                labels=[])
            metrics.append(calc_median_altitude)
            calc_median_longitude = GaugeMetricFamily(
                PROMETHEUS_NAMESPACE + '_calc_median_longitude',
                'Median longitude in m over the last day',
                labels=[])
            metrics.append(calc_median_longitude)
            calc_median_latitude = GaugeMetricFamily(
                PROMETHEUS_NAMESPACE + '_calc_median_latitude',
                'Median latitude in m over the last day',
                labels=[])
            metrics.append(calc_median_latitude)

            medianaltitude = statistics.median(self.lastaltitudevalues)
            altitude_for_sealevel = medianaltitude
            calc_median_altitude.add_metric(labels=[], value=medianaltitude)

            calc_median_longitude.add_metric(labels=[], value=statistics.median(self.lastlongitudevalues))
            calc_median_latitude.add_metric(labels=[], value=statistics.median(self.lastlatitudevalues))

            # reduce queue if full
            if len(self.lastaltitudevalues) >= self.lastaltitudevalues.maxlen:
                self.lastaltitudevalues.popleft()
            if len(self.lastlongitudevalues) >= self.lastlongitudevalues.maxlen:
                self.lastlongitudevalues.popleft()
            if len(self.lastlatitudevalues) >= self.lastlatitudevalues.maxlen:
                self.lastlatitudevalues.popleft()

        if has_altitude and has_pressure:
            calc_pressure_at_sealevel = GaugeMetricFamily(
                PROMETHEUS_NAMESPACE + '_calc_pressure_at_sealevel',
                'Calculated pressure at sealevel in hPa',
                labels=[])
            metrics.append(calc_pressure_at_sealevel)

            sealevel_pressure = self.pressure_to_sealevel_pressure(pressure, altitude_for_sealevel)
            calc_pressure_at_sealevel.add_metric(labels=[], value=sealevel_pressure/100)

        return metrics

class Zgmco2Sensor(object):
    """Read data from a ZG mini CO2 sensor"""

    VID = 0x04d9
    PID = 0xa052

    lasttemperature = None
    lastco2ppm = None
    manufacturer = None
    product = None
    serial = None

    # Thanks to: https://hackaday.io/project/5301-reverse-engineering-a-low-cost-usb-co-monitor
    def decrypt(self, key,  data):
        cstate = [0x48,  0x74,  0x65,  0x6D,  0x70,  0x39,  0x39,  0x65]
        shuffle = [2, 4, 0, 7, 1, 6, 5, 3]

        phase1 = [0] * 8
        for i, o in enumerate(shuffle):
            phase1[o] = data[i]
        phase2 = [0] * 8
        for i in range(8):
            phase2[i] = phase1[i] ^ key[i]
        phase3 = [0] * 8
        for i in range(8):
            phase3[i] = ( (phase2[i] >> 3) | (phase2[ (i-1+8)%8 ] << 5) ) & 0xff
        ctmp = [0] * 8
        for i in range(8):
            ctmp[i] = ( (cstate[i] >> 4) | (cstate[i]<<4) ) & 0xff
        out = [0] * 8
        for i in range(8):
            out[i] = (0x100 + phase3[i] - ctmp[i]) & 0xff
        return out

    def mainloop(self):
        with hid.Device(self.VID, self.PID) as h:
            self.manufacturer = h.manufacturer
            logging.debug(f'Device manufacturer: {self.manufacturer}')
            self.product = h.product
            logging.debug(f'Product: {self.product}')
            self.serial = h.serial
            logging.debug(f'Serial Number: {self.serial}')

            key = (0).to_bytes(8, byteorder='big')
            frdata = b"\x00" +key
            h.send_feature_report(frdata)
            #  h.write(frdata)

            while True:
                data = h.read(200, 10000)
                # logging.debug(f'Read ({type(data)}, {len(data)}): {data}')
                if len(data)!=8:
                    logging.info(f'Unexpected packet received (expected size 8, received {len(data)}.')
                    continue
                decrypteddata = self.decrypt(key, data)
                checksum = (decrypteddata[0] + decrypteddata[1] + decrypteddata[2]) & 0xff
                if checksum != decrypteddata[3]:
                    logging.info(f'Checksum failed (calculated: {checksum} received: {decrypteddata[3]}')
                    continue
                if decrypteddata[4] != 0x0D:
                    logging.info('End of message missing')
                    continue
                logging.debug(f'Decrypted Data: {bytes(decrypteddata)}')
                if decrypteddata[0]==0x42:
                    value = ((decrypteddata[1] << 8) + decrypteddata[2])/16.0 - 273.15
                    self.lasttemperature = value
                    logging.debug(f'Temp = {value} °C')
                if decrypteddata[0]==0x50:
                    value = (decrypteddata[1] << 8) + decrypteddata[2]
                    self.lastco2ppm = value
                    logging.debug(f'CO2 = {value} ppm')


class Gm45Sensor(object):
    """Read data from a GM-45 Geiger counter"""

    device = None
    port = None
    count_lock = threading.Lock()
    count = 0
    initialized = False

    def __init__(self, device):
        self.device = device

        self.port = serial.Serial(self.device, baudrate=38400, timeout=0.0,dsrdtr=True)

    def mainloop(self):
        # throw first events away
        time.sleep(1)
        ignoreeventcount = len(self.port.read(10000)) #read some bytes from the serial port
        time.sleep(1)
        ignoreeventcount += len(self.port.read(10000)) #read some bytes from the serial port

        if ignoreeventcount > 0:
            logging.info(f'GM45: ignoring {ignoreeventcount} old events')

        self.initialized = True
        while True:
            time.sleep(5)
            self.updateCount()

    def updateCount(self):
        if not self.initialized:
            return

        recbytes=self.port.read(1000) #read some bytes from the serial port
        new_counts=len(recbytes) #number of bytes is the number of detection events

        with self.count_lock:
            self.count += new_counts
        logging.debug(f'GM45 count increased by {new_counts}')

if __name__ == '__main__':
    logging.basicConfig(level=os.environ.get("LOGLEVEL", "INFO"))
#    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    PARSER = argparse.ArgumentParser()
    PARSER.add_argument("--port", help="The port where to expose the exporter (default:9999)", default=9999)
    PARSER.add_argument("--use_bmp085", help="Set to true to use the BMP085/BMP180 pressure sensor", default="False")
    PARSER.add_argument("--use_gps", help="Set to true to use gps data from gpsd", default="False")
    PARSER.add_argument("--fixedposition", help="Set to true if the device is at a fixed position (activates long term average for position and height)", default="True")
    #PARSER.add_argument("--use_tsl2561", help="Set to true to use the TSL2561 luminosity sensor", default="False")
    PARSER.add_argument("--use_zgmco2", help="Set to true to use the ZG mini CO2 sensor", default="False")
    PARSER.add_argument("--use_gm45", help="Set to true to use the GM-45 Geiger counter sensor", default="False")
    PARSER.add_argument("--gm45_device", help="Device to use for gm45 (default: /dev/ttyUSB0)", default="/dev/ttyUSB0")
    ARGS = PARSER.parse_args()

    port = int(ARGS.port)
    use_bmp085 = bool(util.strtobool(ARGS.use_bmp085))
    use_gps = bool(util.strtobool(ARGS.use_gps))
    use_zgmco2 = bool(util.strtobool(ARGS.use_zgmco2))
    use_gm45 = bool(util.strtobool(ARGS.use_gm45))
    gm45_device = str(ARGS.gm45_device)
    fixedposition = bool(util.strtobool(ARGS.fixedposition))

    if use_bmp085:
        logging.info("BMP085/BPM180 pressure sensor enabled")
    if use_gps:
        logging.info("GPS data (gpsd) enabled")
    if use_zgmco2:
        logging.info("ZG mini CO2 sensor enabled")
    if use_gm45:
        logging.info("GM-45 Geiger counter enabled")

    SENSORY_COLLECTOR = SensorsCollector(use_bmp085, use_gps, use_zgmco2, use_gm45, gm45_device, fixedposition)

    logging.info("Starting exporter on port {}".format(port))
    prometheus_client.start_http_server(port)

    # sleep indefinitely
    while True:
        time.sleep(60)
