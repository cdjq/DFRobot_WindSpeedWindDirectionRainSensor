DFRobot_WindSpeedWindDirectionRainSensor
===========================

* [English Version](./README.md)

本库提供了获取KIT0192采集数据的全部方法，用户只需要简单的使用本库就可以获取到KIT0192采集的数据。

![产品效果图片](../resources/images/KIT0192.png)

## 产品链接（https://www.dfrobot.com）
    SKU：KIT0192
  
## 目录

  * [概述](#概述)
  * [库安装](#库安装)
  * [方法](#方法)
  * [兼容性](#兼容性)
  * [历史](#历史)
  * [创作者](#创作者)

## 概述
* 读取风速计数据
* 读取风向计数据
* 读取雨量计数据

## 库安装

要使用库, 首先下载库文件, 将其粘贴到指定的目录中, 然后打开examples文件夹并在该文件夹中运行演示。
本库关联了modbus_tk、smbus库，使用前请确保树莓派已经下载了modbus_tk、smbus。

## 方法

```python
  def begin(self)
    '''!
      @brief 本函数将会尝试与从机设备进行通信,根据返回值判断通信是否成功
      @return 返回通信结果
      @retval true  Succeed
      @retval false Failed
    '''

  def get_firmware_version(self)
    '''!
      @brief  get firmware version
      @return  Return  firmware version
    '''

  def get_pid_vid(self)
    '''!
      @brief  获取pid和vid
      @return  Return  true:正确获取，false:获取数据失败或者获取数据错误
    '''

  def get_wind_direction(self)
    '''!
      @brief  获取实时风向
      @return 风向角度 单位：°
    '''

  def get_average_wind_direction(self,miunte)
    '''!
      @brief  获取指定时间内的平均风向
      @param minute 指定时间(有效设置为1-5分钟)
      @return 风向角度 单位：°
    '''

  def get_wind_speed(self)
    '''!
      @brief  获取实时风速
      @return 实时风速 单位:m/s
    '''

  def get_average_wind_speed(self,miunte)
    '''!
      @brief  获取指定时间内的平均风速
      @param minute 指定时间(有效设置为1-5分钟)
      @return 风速 单位:m/s 
    '''

  def get_max_wind_speed(self)
    '''!
      @brief  获取系统运行以来测量的最大风速
      @return 风速 单位:m/s 
    '''

  def get_max_wind_speed_time(self,miunte)
    '''!
      @brief  获取指定时间内测量的最大风速
      @param minute 指定时间(有效设置为1-5分钟)
      @return 风速 单位:m/s 
    '''

  def get_rainfall(self)
    '''!
      @brief  获取累计雨量
      @return 累计雨量
    '''

  def get_rainfall_time(self,hour)
    '''!
      @brief  获取指定时间内的累计雨量
      @param hour 指定时间(有效设置为1-24小时)
      @return 累计雨量
    '''

  def get_raw_data(self,type)
    '''!
      @brief 获取原始数据
      @param type 1:风向传感器的ADC值,2:风速传感器的频率，单位 hz 3:雨量的翻斗次数，单位 次
      @return 风向传感器的ADC值；风速传感器的频率，单位 hz ; 雨量的翻斗次数，单位 次
    '''

  def set_radius_anemometer(self,radius)
    '''!
      @brief 设置风速计半径,数据掉电存储，默认为9
      @param radius 风速计半径
      @return 返回 0 设置成功，其他值设置失败 
    '''

  def set_rain_accumulated_value(self,value)
    '''!
      @brief 设置雨量累加值
      @param value 雨量累加值，单位为毫米
      @return 返回 0 设置成功，其他值设置失败 
    '''
```


## 兼容性

* RaspberryPi 版本

| Board        | Work Well | Work Wrong | Untested | Remarks |
| ------------ | :-------: | :--------: | :------: | ------- |
| RaspberryPi2 |           |            |    √     |         |
| RaspberryPi3 |           |            |    √     |         |
| RaspberryPi4 |     √     |            |          |         |

* Python 版本

| Python  | Work Well | Work Wrong | Untested | Remarks |
| ------- | :-------: | :--------: | :------: | ------- |
| Python2 |     √     |            |          |         |
| Python3 |     √     |            |          |         |


## 历史
- 2022-08-29 - 1.0.0 版本

## 创作者

Written by fary(feng.yang@dfrobot.com), 2022. (Welcome to our [website](https://www.dfrobot.com/))