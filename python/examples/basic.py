# -*- coding: utf-8 -*
'''!
  @file  basic.py
  @brief 本示例介绍了DFRobot_WindSpeedWindDirectionRainSensor库的所有用法
  @copyright   Copyright (c) 2021 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author      [fary](feng.yang@dfrobot.com)
  @version     V1.0
  @date        2022-08-29
  @url         https://github.com/DFRobor/DFRobot_WindSpeedWindDirectionRainSensor
'''
from __future__ import print_function
import sys
import os
sys.path.append("../")
import time

from DFRobot_WindSpeedWindDirectionRainSensor import *

#sensor=DFRobot_WindSpeedWindDirectionRainSensor_UART()
sensor=DFRobot_WindSpeedWindDirectionRainSensor_I2C()

def setup():
  while (sensor.begin() == False):
    print("Sensor initialize failed!!")
    time.sleep(1)
  print("Sensor  initialize success!!")

  print("Version: "+ sensor.get_firmware_version())
  print("vid: %#x"%(sensor.vid))
  print("pid: %#x"%(sensor.pid))
  #设置风速计半径，单位为cm
  sensor.set_radius_anemometer(9)
  #设置雨量累加值，单位为mm
  sensor.set_rain_accumulated_value(0.2794)
def loop():
  #获取实时风向
  wind_direction=sensor.get_wind_direction()
  #获取1分钟内的平均风向(函数参数可选1-5)
  one_minute_wind_direction=sensor.get_average_wind_direction(1)
  #获取实时风速
  wind_speed=sensor.get_wind_speed()
  #获取一分钟内的平均风速(函数参数可选1-5)
  one_minute_wind_speed=sensor.get_average_wind_speed(1)
  #获取系统运行时间的最大风速
  max_wind_speed=sensor.get_max_wind_speed()
  #获取一分钟内的最大风速(函数参数可选1-5)
  one_minute_max_wind_speed=sensor.get_max_wind_speed_time(1)
  #获取系统运行时间内的累计雨量
  rainfall=sensor.get_rainfall()
  #获取系统1小时内的累计雨量（函数参数可选1-24）
  one_hour_rainfall=sensor.get_rainfall_time(1)

  #获取原始数据，1：风向计采集的adc；2：风速传感器的频率，单位 hz；3：雨量的翻斗次数，单位 次
  wind_direction_adc=sensor.get_raw_data(1)
  speed_raw=sensor.get_raw_data(2)
  rainfall_raw=sensor.get_raw_data(3)

  print("wind direction: %f °"%(wind_direction))
  print("Average wind direction in one minute: %f °"%(one_minute_wind_direction))
  print("wind speed: %f m/s"%(wind_speed))
  print("Average wind speed in one minute: %f m/s"%(one_minute_wind_speed))
  print("Max wind speed : %f m/s"%(max_wind_speed))
  print("Max wind speed in one minute: %f m/s"%(one_minute_max_wind_speed))
  print("rainfall : %f mm"%(rainfall))
  print("The amount of rain in one hour: %f mm"%(one_hour_rainfall))

  print("The original value collected by the windometer: %d"%(wind_direction_adc))
  print("Raw values collected by the anemometer : %d"%(speed_raw))
  print("Original value collected by rain gauge %d"%(rainfall_raw))
  print("--------------------------------------------------------------------")
  time.sleep(1)

if __name__ == "__main__":
  setup()
  while True:
    loop()