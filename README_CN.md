DFRobot_WindSpeedWindDirectionRainSensor
===========================

* [English Version](./README.md)

本库提供了获取KIT0192采集数据的全部方法，用户只需要简单的使用本库就可以获取到KIT0192采集的数据。

![产品效果图片](./resources/images/KIT0192.png)

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

这里提供两种使用本库的方法：
1. 打开Arduino IDE,在状态栏中的Tools--->Manager Libraries 搜索"DFRobot_WindSpeedWindDirectionRainSensor"并安装本库.
2. 首先下载库文件,将其粘贴到\Arduino\libraries目录中,然后打开examples文件夹并在该文件夹中运行演示.
注意：本库需要配合DFRobot_RTU使用，确保安装了DFRobot_RTU后再使用本库

## 方法

```C++
  /**
   * @fn begin
   * @brief 本函数将会尝试与从机设备进行通信,根据返回值判断通信是否成功
   * @return 返回通信结果
   * @retval true  Succeed
   * @retval false Failed
   */
  bool begin(void);

  /**
   * @fn getFirmwareVersion
   * @brief  get firmware version
   * @return  Return  firmware version
   */
  String getFirmwareVersion();

  /**
   * @fn getVid
   * @brief  Get VID and PID
   * @return  Return  true:正确获取，false:获取数据失败或者获取数据错误
   */
  bool getPidVid();

  /**
   * @brief 获取实时风向
   * 
   * @return 风向角度 单位：°
   */
  float getWindDirection();

  /**
   * @brief 获取指定时间内的平均风向
   *
   * @param minute 指定时间(有效设置为1-5分钟)
   * @return 风向角度 单位：°
   */
  float getWindDirection(uint8_t minute);

  /**
   * @brief 获取实时风速
   * 
   * @return 实时风速 单位:m/s
   */
  float getWindSpeed();

  /**
   * @brief 获取指定时间内的平均风速
   * 
   * @param minute 指定时间(有效设置为1-5分钟)
   * @return float 
   */
  float getWindSpeed(uint8_t minute);

  /**
   * @brief 获取系统运行以来测量的最大风速
   * 
   * @return 风速 单位:m/s 
   */
  float getMaxWindSpeed();

  /**
   * @brief 获取指定时间内测量的最大风速
   * 
   * @param minute 指定时间(有效设置为1-5分钟)
   * @return 风速 单位:m/s 
   */
  float getMaxWindSpeed(uint8_t minute);

  /**
   * @brief 获取累计雨量
   * 
   * @return float 累计雨量
   */
  float getRainfall();

  /**
   * @brief 获取指定时间内的累计雨量
   * 
   * @param hour 指定时间(有效设置为1-24小时)
   * @return float 累计雨量
   */
  float getRainfall(uint8_t hour);

  /**
   * @brief Get the Rawdata object
   * 
   * @param dataType 1:风向传感器的ADC值,2:风速传感器的频率，单位 hz 3:雨量的翻斗次数，单位 次
   * @return 风向传感器的ADC值；风速传感器的频率，单位 hz ; 雨量的翻斗次数，单位 次
   */
  uint32_t getRawdata(uint8_t dataType);

  /**
   * @brief 设置风速计半径,数据掉电存储，默认为9
   * 
   * @param radius 风速计半径
   * @return 返回 0 设置成功，其他值设置失败 
   */
  uint8_t setRadiusAnemometer(uint8_t radius);

  /**
   * @brief Set the Rain Accumulated Value object
   * 
   * @param accumulatedValue 雨量累加值，单位为毫米
   * @return 返回 0 设置成功，其他值设置失败 
   */
  uint8_t setRainAccumulatedValue(float accumulatedValue);
```


## 兼容性

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | :----:
Arduino Uno        |      √       |              |             |
Arduino MEGA2560   |      √       |              |             |
Arduino Leonardo   |      √       |              |             |
FireBeetle-ESP8266 |      √       |              |             |
FireBeetle-ESP32   |      √       |              |             |
FireBeetle-M0      |      √       |              |             |
Micro:bit          |              |       √      |             |

## 历史
- 2022-08-29 - 1.0.0 版本

## 创作者

Written by fary(feng.yang@dfrobot.com), 2022. (Welcome to our [website](https://www.dfrobot.com/))