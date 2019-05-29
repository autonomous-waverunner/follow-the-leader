#!/usr/bin/env python
# -*- coding: utf-8 -*-

# for voltage/power: /sys/bus/i2c/drivers/ina3221x/0-0040/iio_device/in_
# for temp: /sys/devices/virtual/thermal/.../temp
# for test: getSystemInformation_test_files/

# Program to print all critical data to ensure that the system operates correctly.

from time import sleep
import datetime

test = False

vp_path = './getSystemInformation_test_files/{}' if test else '/sys/bus/i2c/drivers/ina3221x/1-0040/iio_device/in_{}'
th_path = './getSystemInformation_test_files/{}' if test else '/sys/devices/virtual/thermal/{}/temp'

voltage_power_files = {
    'gpuVoltage': 'voltage0_input',
    'gpuPower': 'power0_input',
    'cpuVoltage': 'voltage1_input',
    'cpuPower': 'power1_input',
    'socVoltage': 'voltage2_input',
    'socPower': 'power2_input'
}

thermal_files = {
    'gpuTemp': 'thermal_zone0',
    'cpuTemp': 'thermal_zone1',
    'socTemp': 'thermal_zone2',
}

def read_file(fname):
    with open(fname) as f:
        data = int(f.read())
    return data

def get_values():
    voltage_power_values = { k: read_file(vp_path.format(v)) for k, v in voltage_power_files.items() }
    thermal_values = { k: read_file(th_path.format(v)) / 1000 for k, v in thermal_files.items() }
    return { **voltage_power_values, **thermal_values }

def values_string(values):
    return '''GPU: \t{gpuVoltage} (mV), \t{gpuPower} (mW), \t{gpuTemp} °C
CPU: \t{cpuVoltage} (mV), \t{cpuPower} (mW), \t{cpuTemp} °C
SOC: \t{socVoltage} (mV), \t{socPower} (mW), \t{socTemp} °C'''.format(**values)

#if __name__ == '__main__':
while True:
    data = get_values()
    now = datetime.datetime.now()
    print('{}:{}:{},'.format(now.hour, now.minute, now.second), end='')
    print('{gpuTemp},{cpuTemp},{socTemp}\n'.format(**data))
    sleep(1)
    #values = get_values()
    #print(values_string(values))
