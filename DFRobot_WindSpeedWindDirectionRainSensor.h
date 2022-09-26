/*!
 * @file  DFRobot_WindSpeedWindDirectionRainSensor.h
 * @brief  Define infrastructure of DFRobot_WindSpeedWindDirectionRainSensor class
 * @details  该库实现了与Kit0192设备进行通信的所有功能，包括配置设备参数和读取设备数据
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [fary](feng.yang@dfrobot.com)
 * @version  V1.0
 * @date  2022-07-14
 * @url  https://github.com/DFRobot/DFRobot_WindSpeedWindDirectionRainSensor
 */
#ifndef __DFROBOT_WINDSPEEDWINFDIRECTIONRAINSENSOR_H__
#define __DFROBOT_WINDSPEEDWINFDIRECTIONRAINSENSOR_H__
#include <Arduino.h>
#include "DFRobot_RTU.h"
#include "Wire.h"
#define IIC_MODE  0
#define UART_MODE 1
class DFRobot_WindSpeedWindDirectionRainSensor{
public: 
  /**
   * @enum  eKit0192InputReg_t
   * @brief  设备的输入寄存器地址
   */
  typedef enum{
    eInputRegPidKit0192=0x0000,                 /**< Kit0192 存储PID的输入寄存器的地址 */
    eInputRegVidKit0192,                        /**< Kit0192 存储VID的输入寄存器的地址 */
    eInputRegRegAddrKit0192,                    /**< Kit0192 储存设备地址的寄存器 */
    eInputRegBaudKit0192,                       /**< Kit0192 储存串口波特率的寄存器 */
    eInputRegVerifyAndStopKit0192,              /**< Kit0192 存储串口奇偶检验位与停止位的输入寄存器的地址 */
    eInputRegVersionKit0192,                    /**< Kit0192 存储固件版本的输入寄存器地址 */
    eInputRegAverageWindDirectionKit0192,       /**< Kit0192 存储平均风向的寄存器 */
    eInputRegRealTimeWindDirectionKit0192,      /**< Kit0192 存储实时风向的寄存器 */
    eInputRegAverageWindSpeedKit0192,           /**< Kit0192 存储平均风速的寄存器 */
    eInputRegRealTimeWindSpeedKit0192,          /**< Kit0192 存储实时风向的寄存器 */
    eInputRegMaxWindSpeedMinuteKit0192,         /**< Kit0192 存储设置时间内的最大风速寄存器 */
    eInputRegMaxWindSpeedKit0192,               /**< Kit0192 存储开始工作后最大风速寄存器 */
    eInputRegTimeRainFallLKit0192,              /**< Kit0192 存储设置时间内的累计雨量低16位 */
    eInputRegTimeRainFallHKit0192,              /**< Kit0192 存储设置时间内的累计雨量高16位 */
    eInputRegCumulativeRainFallLKit0192,        /**< Kit0192 存储开始工作后累计雨量低16位 */
    eInputRegCumulativeRainFallHKit0192,        /**< Kit0192 存储开始工作后累计雨量高16位 */
    eInputRegRawDataLKit0192,                   /**< Kit0192 存储原始数据低16位 */
    eInputRegRawDataHKit0192,                   /**< Kit0192 存储原始数据高16位 */
  }eKit0192InputReg_t;

  /**
   * @enum  eKit0192HoldingReg_t
   * @brief  设备的保持寄存器地址
   */
  typedef enum{
    eHoldingRegReserved0Kit0192=0x0000,         /**< Kit0192 该寄存器保留 */
    eHoldingRegReserved1Kit0192,                /**< Kit0192 该寄存器保留 */
    eHoldingRegReserved2Kit0192,                /**< Kit0192 该寄存器保留 */
    eHoldingRegReserved3Kit0192,                /**< Kit0192 该寄存器保留 */
    eHoldingRegReserved4Kit0192,                /**< Kit0192 该寄存器保留 */
    eHoldingRegReserved5Kit0192,                /**< Kit0192 该寄存器保留 */
    eHoldingRegWindDirectionMinuteKit0192,      /**< Kit0192 设置计算平均风向的时间 */
    eHoldingRegWindSpeedAverageMinuteKit0192,   /**< Kit0192 设置计算平均风速的时间 */
    eHoldingRegWindSpeedMaxMinuteKit0192,       /**< Kit0192 设置计算最大风速的时间 */
    eHoldingRegRainHourKit0192,                 /**< Kit0192 设置计算累计雨量的时间 */
    eHoldingRegRadiusAnemometerKit0192,         /**< Kit0192 设置风速计半径 */
    eHoldingRegBaseRainFallKit0192,             /**< Kit0192 设置雨量累加值 */
    eHoldingRegRawTypeKit0192,                  /**< Kit0192 设置原始数据类型 */
  }eKit0192HoldingReg_t;

  /**
   * @enum  eKit0192I2cReg_t
   * @brief  设备的i2c寄存器地址，与串口寄存器一致，但是串口寄存器是16位，所以这里进行拆分为8位
   */
  typedef enum{
    eI2cRegPidL=0x0000,
    eI2cRegPidH,
    eI2cRegVidL,
    eI2cRegVidH,
    eI2cRegRegAddrL,
    eI2cRegRegAddrH,
    eI2cRegBaudL,
    eI2cRegBaudH,
    eI2cRegVerifyAndStopL,
    eI2cRegVerifyAndStopH,
    eI2cRegVersionL,
    eI2cRegVersionH,
    eI2cRegAverageWindDirectionL,
    eI2cRegAverageWindDirectionH,
    eI2cRegRealTimeWindDirectionL,
    eI2cRegRealTimeWindDirectionH,
    eI2cRegAverageWindSpeedL,
    eI2cRegAverageWindSpeedH,
    eI2cRegRealTimeWindSpeedL,
    eI2cRegRealTimeWindSpeedH,
    eI2cRegMaxWindSpeedMinuteL,
    eI2cRegMaxWindSpeedMinuteH,
    eI2cRegMaxWindSpeedL,
    eI2cRegMaxWindSpeedH,
    eI2cRegTimeRainFallLL,
    eI2cRegTimeRainFallLH,
    eI2cRegTimeRainFallHL,
    eI2cRegTimeRainFallHH,
    eI2cRegCumulativeRainFallLL,
    eI2cRegCumulativeRainFallLH,
    eI2cRegCumulativeRainFallHL,
    eI2cRegCumulativeRainFallHH,
    eI2cRegRawDataLL,
    eI2cRegRawDataLH,
    eI2cRegRawDataHL,
    eI2cRegRawDataHH,
    eI2cRegReserved0L,
    eI2cRegReserved0H,
    eI2cRegReserved1L,
    eI2cRegReserved1H,
    eI2cRegReserved2L,
    eI2cRegReserved2H,
    eI2cRegReserved3L,
    eI2cRegReserved3H,
    eI2cRegReserved4L,
    eI2cRegReserved4H,
    eI2cRegReserved5L,
    eI2cRegReserved5H,
    eI2cRegWindDirectionMinuteL,
    eI2cRegWindDirectionMinuteH,
    eI2cRegWindSpeedAverageMinuteL,
    eI2cRegWindSpeedAverageMinuteH,
    eI2cRegWindSpeedMaxMinuteL,
    eI2cRegWindSpeedMaxMinuteH,
    eI2cRegRainHourL,
    eI2cRegRainHourH,
    eI2cRegRadiusAnemometerL,
    eI2cRegRadiusAnemometerH,
    eI2cRegBaseRainFallL,
    eI2cRegBaseRainFallH,
    eI2cRegRawTypeL,
    eI2cRegRawTypeH,
  }eKit0192I2cReg_t;

  /**
   * @brief Construct a new dfrobot windspeedwinddirectionrainsensor object
   * 
   * @param mode 工作模式，IIC_MODE:0 ,UART_MODE:1
   */
  DFRobot_WindSpeedWindDirectionRainSensor(uint8_t mode);
  ~DFRobot_WindSpeedWindDirectionRainSensor(){};

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
  String getFirmwareVersion(void);

  /**
   * @brief 获取实时风向
   * 
   * @return 风向角度 单位：°
   */
  float getWindDirection(void);

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
  float getWindSpeed(void);

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
  float getMaxWindSpeed(void);

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
  float getRainfall(void);

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
  uint8_t setRainAccumulatedValue(float accumulatedValue = 0.2794);
  uint32_t vid;
  uint32_t pid;
private:
  /**
   * @fn getVid
   * @brief  Get VID and PID
   * @return  Return  true:正确获取，false:获取数据失败或者获取数据错误
   */
  bool getPidVid(void);
  virtual uint8_t readRegister(uint8_t reg,void* pBuf, size_t size){};
  virtual uint16_t readRegister(uint16_t reg){};
  virtual uint8_t writeRegister(uint8_t reg,void* pBuf,size_t size){};
  virtual uint16_t writeRegister(uint16_t reg,uint16_t data){};
protected:

private:
  int    _dePin;
  uint8_t   _mode;
};
class DFRobot_WindSpeedWindDirectionRainSensor_UART:public DFRobot_WindSpeedWindDirectionRainSensor,public DFRobot_RTU{
public: 
  /**
   * @brief Construct a new dfrobot windspeedwinddirectionrainsensor uart object
   * 
   * @param s 需要使用的串口设备
   */
  DFRobot_WindSpeedWindDirectionRainSensor_UART(Stream *s);
  ~DFRobot_WindSpeedWindDirectionRainSensor_UART(){};
  /**
   * @fn begin
   * @brief 本函数将会尝试与从机设备进行通信,根据返回值判断通信是否成功
   * @return 返回通信结果
   * @retval true  Succeed
   * @retval false Failed
   */
  bool begin(void);
private:
  /**
   * @brief 读输入寄存器
   * 
   * @param reg 输入寄存器地址
   * @return uint16_t 读到的数据
   */
  uint16_t readRegister(uint16_t reg);

  /**
   * @brief 写保持寄存器,写完后内部延时了12MS,传感器需要下写入后花费12Ms进行保存，此时不能进行通信
   * 
   * @param reg 保持寄存器地址
   * @param data 要写入的数据
   * @return uint16_t 写入结果,0表示成功，其他值表示失败
   */
  uint16_t writeRegister(uint16_t reg,uint16_t data);
private:
  uint8_t _deviceAddr;
};

class DFRobot_WindSpeedWindDirectionRainSensor_IIC:public DFRobot_WindSpeedWindDirectionRainSensor{
public: 
  /**
   * @brief Construct a new dfrobot windspeedwinddirectionrainsensor iic object
   * 
   * @param pWire 需要使用的I2C设备
   */
  DFRobot_WindSpeedWindDirectionRainSensor_IIC(TwoWire *pWire);
  ~DFRobot_WindSpeedWindDirectionRainSensor_IIC(){};
  /**
   * @fn begin
   * @brief 本函数将会尝试与从机设备进行通信,根据返回值判断通信是否成功
   * @return 返回通信结果
   * @retval true  Succeed
   * @retval false Failed
   */
  bool begin(void);
private:
  /**
   * @brief 读I2C寄存器
   * 
   * @param reg I2C寄存器地址
   * @param pBuf 缓存空间
   * @param size 读取长度
   * @return 读取长度
   */
  uint8_t readRegister(uint8_t reg,void* pBuf, size_t size);

  /**
   * @brief 写I2C寄存器,写完后内部延时了12MS,传感器需要下写入后花费12Ms进行保存，此时不能进行通信
   * 
   * @param reg I2C寄存器地址
   * @param pBuf 数据存储空间
   * @param size 读取长度
   * @return uint8_t 0表示成功，其他值表示失败
   */
  uint8_t writeRegister(uint8_t reg,void* pBuf,size_t size);
private:
  TwoWire* _pWire;
  uint8_t _deviceAddr;
};
#endif