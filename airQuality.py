#!/usr/bin/env python3
"""
@brief   Air Quality Monitoring using the CCS811 sensor.
@file    air_quality_monitor.py
"""

import time
import RPi.GPIO as GPIO
from smbus2 import SMBus
import threading

# Use the Broadcom SOC channel's pin numbers
GPIO.setmode(GPIO.BCM)

# Set up the GPIO pin 17 as an output
GPIO.setup(17, GPIO.OUT)

# CCS811 Addresses
CCS811_ADDRESS = 0x5A
CCS811_STATUS = 0x00
CCS811_ALG_RESULT_DATA = 0x02

# Start the I2C bus
bus = SMBus(1)

# Create a global variable to hold sensor data
sensor_data = {'co2': 0, 'tvoc': 0, 'ready': False}

def calculate_aqi(co2, tvoc):
    """
    @brief   Calculates AQI based on the given CO2 and TVOC values.
    @param   co2 The CO2 value.
    @param   tvoc The TVOC value.
    @return  A tuple containing the AQI value and the air quality status.
    @author  Wenyuan Yu
    @date    2023-08-09
    """
    co2_breakpoints = [(0, 1000), (1001, 2000), (2001, 4000), (4001, 8192)]
    tvoc_breakpoints = [(0, 200), (201, 500), (501, 800), (801, 1187)]
    aqi_breakpoints = [(0, 50), (51, 100), (101, 150), (151, 200)]

    def interpolate(value, low1, high1, low2, high2):
        return ((value - low1) / (high1 - low1)) * (high2 - low2) + low2

    co2_aqi = tvoc_aqi = 0
    for low, high in co2_breakpoints:
        if low <= co2 <= high:
            co2_aqi = interpolate(co2, low, high, *aqi_breakpoints[co2_breakpoints.index((low, high))])
            break

    for low, high in tvoc_breakpoints:
        if low <= tvoc <= high:
            tvoc_aqi = interpolate(tvoc, low, high, *aqi_breakpoints[tvoc_breakpoints.index((low, high))])
            break

    combined_aqi = max(co2_aqi, tvoc_aqi)

    if combined_aqi <= 50:
        return (int(combined_aqi), "Good")
    elif combined_aqi <= 100:
        return (int(combined_aqi), "Moderate")
    elif combined_aqi <= 150:
        return (int(combined_aqi), "Poor")
    else:
        return (int(combined_aqi), "Unhealthy")

def read_sensor_data():
    """
    @brief   Continuously reads and updates the sensor data.
    @details Reads data from the CCS811 sensor, calculates CO2 and TVOC values,
             and updates the global sensor_data dictionary.
    @author  Wenyuan Yu
    @date    2023-08-09
    """
    global sensor_data
    while True:
        try:
            status = bus.read_byte_data(CCS811_ADDRESS, CCS811_STATUS)
            if status & 0x08:
                data = bus.read_i2c_block_data(CCS811_ADDRESS, CCS811_ALG_RESULT_DATA, 8)
                co2 = (data[0] << 8) | data[1]
                tvoc = (data[2] << 8) | data[3]
                sensor_data = {'co2': co2, 'tvoc': tvoc, 'ready': True}
            time.sleep(1)
        except Exception as e:
            print("Sensor thread exception: ", str(e))
            break

def handle_gpio():
    """
    @brief   Manages the GPIO based on the sensor readings.
    @details Prints the AQI value and air quality status. Flashes an LED 
             if the air quality is unhealthy.
    @author  Wenyuan Yu
    @date    2023-08-09
    """
    global sensor_data
    while True:
        if sensor_data['ready']:
            co2 = sensor_data['co2']
            tvoc = sensor_data['tvoc']
            aqi_num, aqi_status = calculate_aqi(co2, tvoc)
            print(f"AQI: {aqi_num}, Air quality: {aqi_status}")
            
            if aqi_status == "Unhealthy":
                GPIO.output(17, GPIO.HIGH)
                print("Alert! Air quality is terrible. Please take necessary steps to improve the AQI.")
                time.sleep(1)
                GPIO.output(17, GPIO.LOW)
                
        time.sleep(1)

# Start the threads
sensor_thread = threading.Thread(target=read_sensor_data)
gpio_thread = threading.Thread(target=handle_gpio)

sensor_thread.start()
gpio_thread.start()

# Wait for the threads to finish
sensor_thread.join()
gpio_thread.join()
