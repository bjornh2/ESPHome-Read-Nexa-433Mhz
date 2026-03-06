# ESPHome to Home Assistant Nexa Temperature Sensor Documentation

## Overview
This integration reads temperature data from a Nexa wireless temperature sensor and reports it to Home Assistant through ESPHome.

## Features
- Reads temperature values from Nexa temperature sensor
- Transmits data wirelessly to ESPHome device
- Integrates seamlessly with Home Assistant
- Real-time temperature monitoring

## Configuration
Configure the ESPHome device to receive and process Nexa sensor signals, then expose the temperature entity to Home Assistant.

## Supported Sensors
- Nexa temperature sensors (compatible 433 MHz RF protocol)

## Home Assistant Integration
Once configured, the temperature sensor will appear as a climate/sensor entity in Home Assistant, allowing for automation, monitoring, and historical data tracking.

## Notes
- Ensure proper RF reception range
- Home Assistant must have ESPHome integration enabled
- Check signal strength if readings are inconsistent