# sensors_exporter

This is a [Prometheus exporter](https://prometheus.io/docs/instrumenting/exporters/) for different sensors.

At the moment supported are:
* [BMP085/BMP180 pressure sensor](https://www.adafruit.com/product/1603)
* [GPS (via GPSd)](https://gpsd.gitlab.io/gpsd/)
* [ZG mini CO2 sensor](https://www.co2meter.com/products/co2mini-co2-indoor-air-quality-monitor) (also known as: [TFA-Dostmann CO2 Monitor AIRCO2NTROL MINI](https://www.tfa-dostmann.de/en/product/co2-monitor-airco2ntrol-mini-31-5006/) and [TFA CO2 Messgerät](https://de.elv.com/tfa-co2-messgeraet-aircontrol-mini-119661))
* [GM-45 geiger counter](https://www.blackcatsystems.com/GM/raspberry_pi_radiation_detector.html)

It is tested with Raspberry Pis, and the `setup.sh` script is specific for [Raspberry Pi OS Lite](https://www.raspberrypi.org/software/),
but should be simple to adapt for other OSs.

## Usage

Clone the respoitory und install with:
```bash
git clone https://github.com/neredera/sensors_exporter.git
cd sensors_exporter
.\setup.sh
```

Enable/disable the sensors you need via:
```bash
nano sensors_exporter.service

sudo systemctl daemon-reload
sudo systemctl restart sensors_exporter.service
sudo systemctl status sensors_exporter.service
```

Command line parameters:
```bash
> python3 exporter.py --help

usage: exporter.py [-h] [--port PORT] [--use_bmp085 USE_BMP085]
                   [--use_gps USE_GPS] [--fixedposition FIXEDPOSITION]
                   [--use_zgmco2 USE_ZGMCO2]

optional arguments:
  -h, --help            show this help message and exit
  --port PORT           The port where to expose the exporter (default:9999)
  --use_bmp085 USE_BMP085
                        Set to true to use the BMP085/BMP180 pressure sensor
  --use_gps USE_GPS     Set to true to use gps data from gpsd
  --fixedposition FIXEDPOSITION
                        Set to true if the device is at a fixed position
                        (activates long term average for position and height)
  --use_zgmco2 USE_ZGMCO2
                        Set to true to use the ZG mini CO2 sensor
                        
```

## Prometheus metrics

Example how to add the exporter to the prometheus configuration (`prometheus.yml`):
```yml
  - job_name: sensors
    static_configs:
    - targets:
      - rpi-ntp.local:9999
      - rpi-plane.local:9999
      - rpi-co2.local:9999
```

Some sample metrics for the sensors:

### BMP085 pressure sensor

```
# HELP sensor_bmp085_temperature Temperature of pressure sensor bmp085/bmp180 in °C
# TYPE sensor_bmp085_temperature gauge
sensor_bmp085_temperature 15.4
# HELP sensor_bmp085_pressure Pressure of pressure sensor bmp085/bmp180 in hPa
# TYPE sensor_bmp085_pressure gauge
sensor_bmp085_pressure 997.08
```

If you have GPS and BMP the pressure at sea level is calculated from the height:
```
# HELP sensor_calc_pressure_at_sealevel Calculated pressure at sealevel in hPa
# TYPE sensor_calc_pressure_at_sealevel gauge
sensor_calc_pressure_at_sealevel 1021.2032
```

### GPS

With `--fixedposition` active (default) median values for the postion and altitude are calculated over the last 24h (assuming a 15s scarpe interval).
Disable this option if your GPS receiver is mobile.

```
# HELP sensor_gps_mode Status of GPS reception. 0=No value, 1=No fix, 2=2D fix, 3=3D fix
# TYPE sensor_gps_mode gauge
sensor_gps_mode 3.0
# HELP sensor_gps_satellites Number of satellites visible
# TYPE sensor_gps_satellites gauge
sensor_gps_satellites 12.0
# HELP sensor_gps_satellites_used Number of satellites used
# TYPE sensor_gps_satellites_used gauge
sensor_gps_satellites_used 8.0
# HELP sensor_gps_altitude Altitude in m
# TYPE sensor_gps_altitude gauge
sensor_gps_altitude 243.5
# HELP sensor_gps_altitude_error Estimated altitude error (95% confidence) in m
# TYPE sensor_gps_altitude_error gauge
sensor_gps_altitude_error 10.317
# HELP sensor_gps_longitude Longitude
# TYPE sensor_gps_longitude gauge
sensor_gps_longitude 0.00000
# HELP sensor_gps_longitude_error Estimated longitude error (95% confidence) in m
# TYPE sensor_gps_longitude_error gauge
sensor_gps_longitude_error 3.445
# HELP sensor_gps_latitude Latitude
# TYPE sensor_gps_latitude gauge
sensor_gps_latitude 0.00000
# HELP sensor_gps_latitude_error Estimated latitude error (95% confidence) in m
# TYPE sensor_gps_latitude_error gauge
sensor_gps_latitude_error 4.388
# HELP sensor_gps_time_error Estimated time error (95% confidence) in s (not delivered by all receivers)
# TYPE sensor_gps_time_error gauge
sensor_gps_time_error 0.005
# HELP sensor_gps_info_info GPS receiver information
# TYPE sensor_gps_info_info gauge
sensor_gps_info_info{driver="NMEA0183",path="/dev/ttyAMA0",speed="9600"} 1.0
# HELP sensor_calc_median_altitude Median altitude in m over the last day
# TYPE sensor_calc_median_altitude gauge
sensor_calc_median_altitude 243.23
# HELP sensor_calc_median_longitude Median longitude in m over the last day
# TYPE sensor_calc_median_longitude gauge
sensor_calc_median_longitude 0.00000
# HELP sensor_calc_median_latitude Median latitude in m over the last day
# TYPE sensor_calc_median_latitude gauge
sensor_calc_median_latitude 0.00000
```

### ZG mini CO2 sensor

```
# HELP sensor_zgmco2_temperature Temperature of ZG mini CO2 sensor in °C
# TYPE sensor_zgmco2_temperature gauge
sensor_zgmco2_temperature 21.6
# HELP sensor_zgmco2_co2ppm CO2 in ppm from ZG mini CO2 sensor
# TYPE sensor_zgmco2_co2ppm gauge
sensor_zgmco2_co2ppm 734.0
# HELP sensor_zgmco2_info_info ZG mini CO2 sensor information
# TYPE sensor_zgmco2_info_info gauge
sensor_zgmco2_info_info{manufacturer="Holtek",product="USB-zyTemp",serial="1.40"} 1.0
```
