/*!
 * @file  DFRobot_WindSpeedWindDirectionRainSensor.cpp
 * @brief  Define infrastructure of DFRobot_WindSpeedWindDirectionRainSensor class
 * @details  该库实现了与Kit0192设备进行通信的所有功能，包括配置设备参数和读取设备数据
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [fary](feng.yang@dfrobot.com)
 * @version  V1.0
 * @date  2022-08-23
 * @url  https://github.com/DFRobot/DFRobot_WindSpeedWindDirectionRainSensor
 */
#include "DFRobot_WindSpeedWindDirectionRainSensor.h"

DFRobot_WindSpeedWindDirectionRainSensor:: DFRobot_WindSpeedWindDirectionRainSensor(uint8_t mode){
  _mode = mode;
  pid=0;
  vid=0;
}

bool DFRobot_WindSpeedWindDirectionRainSensor::begin()
{
  return getPidVid();
}

String DFRobot_WindSpeedWindDirectionRainSensor::getFirmwareVersion()
{
  uint16_t version=0;
  if(_mode==IIC_MODE){
    uint8_t buff[2]={0};
    readRegister(eI2cRegVersionL,(void*)buff,2);
    version=buff[0]|(((uint16_t)buff[1])<<8);
  }else{
    version =readRegister(eInputRegVersionKit0192);
  }
  return String(version>>12)+'.'+String(((version>>8)&0x0F))+'.'+String(((version>>4)&0x0F))+'.'+String((version&0x0F));
}

bool DFRobot_WindSpeedWindDirectionRainSensor::getPidVid()
{
  bool ret = false;
  if(_mode==IIC_MODE){
    uint8_t buff[4]={0};
    readRegister(eI2cRegPidL,(void*)buff,  4);
    pid = buff[0] | (((uint16_t)buff[1])<<8) | (((uint32_t)(buff[3]&0xC0))<<10);
    vid = buff[2] | (uint16_t)((buff[3]&0x3F)<<8);
  }else{
    pid =readRegister(eInputRegPidKit0192);
    vid =readRegister(eInputRegVidKit0192);
    pid = (vid&0xC000)<<2 | pid;
    vid = vid&0x3FFF;
  }
  if( (vid==0x3343) && (pid == 0x100C0) ){
    ret = true;
  }
  return ret;
}

float DFRobot_WindSpeedWindDirectionRainSensor::getWindDirection()
{
  uint16_t windDirection=0;
  if(_mode==IIC_MODE){
    uint8_t buff[2]={0};
    readRegister(eI2cRegRealTimeWindDirectionL,(void*)buff,2);
    windDirection=buff[0]|(((uint16_t)buff[1])<<8);
  }else{
    windDirection =readRegister(eInputRegRealTimeWindDirectionKit0192);
  }
  return windDirection/10.0;
}

float DFRobot_WindSpeedWindDirectionRainSensor::getWindDirection(uint8_t minute)
{
  if(minute>5)
    return 0;
  uint16_t windDirection=0;
  if(_mode==IIC_MODE){
    writeRegister(eI2cRegWindDirectionMinuteL,(void*)&minute,1);
    uint8_t buff[2]={0};
    readRegister(eI2cRegAverageWindDirectionL,(void*)buff,2);
    windDirection=buff[0]|(((uint16_t)buff[1])<<8);
  }else{
    writeRegister(eHoldingRegWindDirectionMinuteKit0192,minute);
    windDirection =readRegister(eInputRegAverageWindDirectionKit0192);
  }
  return windDirection/10.0;
}

float DFRobot_WindSpeedWindDirectionRainSensor::getWindSpeed()
{
  uint16_t windSpeed=0;
  if(_mode==IIC_MODE){
    uint8_t buff[2]={0};
    readRegister(eI2cRegRealTimeWindSpeedL,(void*)buff,2);
    windSpeed=buff[0]|(((uint16_t)buff[1])<<8);
    
  }else{
    windSpeed =readRegister(eInputRegRealTimeWindSpeedKit0192);
  }
  return windSpeed/100.0;
}

float DFRobot_WindSpeedWindDirectionRainSensor::getWindSpeed(uint8_t miunte)
{
  uint16_t windSpeed=0;
  if(_mode==IIC_MODE){
    writeRegister(eI2cRegWindSpeedAverageMinuteL,(void*)&miunte,1);
    uint8_t buff[2]={0};
    readRegister(eI2cRegAverageWindSpeedL,(void*)buff,2);
    windSpeed=buff[0]|(((uint16_t)buff[1])<<8);
    
  }else{
    writeRegister(eHoldingRegWindSpeedAverageMinuteKit0192,miunte);
    windSpeed =readRegister(eInputRegAverageWindSpeedKit0192);
  }
  return windSpeed/100.0;
}

float DFRobot_WindSpeedWindDirectionRainSensor::getMaxWindSpeed()
{
  uint16_t windSpeed=0;
  if(_mode==IIC_MODE){
    uint8_t buff[2]={0};
    readRegister(eI2cRegMaxWindSpeedL,(void*)buff,2);
    windSpeed=buff[0]|(((uint16_t)buff[1])<<8);
  }else{
    windSpeed =readRegister(eInputRegMaxWindSpeedKit0192);
  }
  return windSpeed/100.0;
}

float DFRobot_WindSpeedWindDirectionRainSensor::getMaxWindSpeed(uint8_t miunte)
{
  uint16_t windSpeed=0;
  if(_mode==IIC_MODE){
    writeRegister(eI2cRegWindSpeedMaxMinuteL,(void*)&miunte,1);
    uint8_t buff[2]={0};
    readRegister(eI2cRegMaxWindSpeedMinuteL,(void*)buff,2);
    windSpeed=buff[0]|(((uint16_t)buff[1])<<8);
  }else{
    writeRegister(eHoldingRegWindSpeedMaxMinuteKit0192,miunte);
    windSpeed =readRegister(eInputRegMaxWindSpeedMinuteKit0192);
  }
  return windSpeed/100.0;
}

float DFRobot_WindSpeedWindDirectionRainSensor::getRainfall()
{
  uint32_t rainfall=0;
  if(_mode==IIC_MODE){
    uint8_t buff[4]={0};
    readRegister(eI2cRegCumulativeRainFallLL,(void*)buff,4);
    rainfall=buff[0]|(((uint32_t)buff[1])<<8)|(((uint32_t)buff[2])<<16)|(((uint32_t)buff[3])<<24);
  }else{
    rainfall =readRegister(eInputRegCumulativeRainFallHKit0192);
    rainfall =rainfall<<16 | readRegister(eInputRegCumulativeRainFallLKit0192);
  }
  return rainfall/10000.0;
}

float DFRobot_WindSpeedWindDirectionRainSensor::getRainfall(uint8_t hour)
{
  uint32_t rainfall=0;
  if(_mode==IIC_MODE){
    writeRegister(eI2cRegRainHourL,(void*)&hour,1);
    uint8_t buff[2]={0};
    readRegister(eI2cRegTimeRainFallLL,(void*)buff,4);
    rainfall=buff[0]|(((uint32_t)buff[1])<<8)|(((uint32_t)buff[2])<<16)|(((uint32_t)buff[3])<<24);
    
  }else{
    writeRegister(eHoldingRegRainHourKit0192,hour);
    rainfall =readRegister(eInputRegTimeRainFallHKit0192);
    rainfall =rainfall<<16 | readRegister(eInputRegTimeRainFallLKit0192);
  }
  return rainfall/10000.0;
}

uint32_t DFRobot_WindSpeedWindDirectionRainSensor::getRawdata(uint8_t type)
{
  uint32_t rawdata=0;
  if(_mode==IIC_MODE){
    writeRegister(eI2cRegRawTypeL,(void*)&type,1);
    uint8_t buff[4]={0};
    readRegister(eI2cRegRawDataLL,(void*)buff,4);
    rawdata=buff[0]|(((uint32_t)buff[1])<<8)|(((uint32_t)buff[2])<<16)|(((uint32_t)buff[3])<<24);
  }else{
    writeRegister(eHoldingRegRawTypeKit0192,type);
    rawdata =readRegister(eInputRegRawDataHKit0192);
    rawdata =rawdata<<16 | readRegister(eInputRegRawDataLKit0192);
  }
  return rawdata;
}

uint8_t DFRobot_WindSpeedWindDirectionRainSensor::setRadiusAnemometer(uint8_t radius)
{
  uint8_t ret =0;
  if(_mode==IIC_MODE){
    ret=writeRegister(eI2cRegRadiusAnemometerL,(void*)&radius,1);
  }else{
    ret=writeRegister(eHoldingRegRadiusAnemometerKit0192,radius);
  }
  return ret;
}

uint8_t DFRobot_WindSpeedWindDirectionRainSensor::setRainAccumulatedValue(float value)
{
  uint8_t ret =0;
  uint16_t data = value*10000;
  if(_mode==IIC_MODE){
    uint8_t buff[2]={0};
    buff[0]=(data&0xFF);
    buff[1]=(data>>8);
    ret=writeRegister(eI2cRegBaseRainFallL,(void*)buff,2);
  }else{
    ret=writeRegister(eHoldingRegBaseRainFallKit0192,data);
  }
  return ret;
}

DFRobot_WindSpeedWindDirectionRainSensor_UART::DFRobot_WindSpeedWindDirectionRainSensor_UART(Stream *s)
:DFRobot_WindSpeedWindDirectionRainSensor(UART_MODE),DFRobot_RTU(s)
{
  _deviceAddr=0xC0;
}
bool DFRobot_WindSpeedWindDirectionRainSensor_UART::begin(void)
{
  return DFRobot_WindSpeedWindDirectionRainSensor::begin();
}
uint16_t DFRobot_WindSpeedWindDirectionRainSensor_UART::readRegister(uint16_t reg)
{
  setTimeoutTimeMs(1000);
  return readInputRegister((uint8_t)_deviceAddr, (uint16_t)reg);
}

uint16_t DFRobot_WindSpeedWindDirectionRainSensor_UART::writeRegister(uint16_t reg,uint16_t data)
{
  uint16_t ret=writeHoldingRegister(_deviceAddr, reg, data);
  delay(12);
  return ret;
}


DFRobot_WindSpeedWindDirectionRainSensor_IIC::DFRobot_WindSpeedWindDirectionRainSensor_IIC(TwoWire *pWire)
:DFRobot_WindSpeedWindDirectionRainSensor(IIC_MODE),_pWire(pWire)
{
  _deviceAddr=0x1D;
}
bool DFRobot_WindSpeedWindDirectionRainSensor_IIC::begin(void)
{
  _pWire->begin();
  return DFRobot_WindSpeedWindDirectionRainSensor::begin();
}

uint8_t DFRobot_WindSpeedWindDirectionRainSensor_IIC::writeRegister(uint8_t reg,void* pBuf,size_t size)
{
  if(pBuf == NULL){
	  return 1;
  }
  uint8_t * _pBuf = (uint8_t *)pBuf;
  _pWire->beginTransmission(_deviceAddr);
  _pWire->write(reg);
  for(uint16_t i = 0; i < size; i++){
    _pWire->write(_pBuf[i]);
  }
  _pWire->endTransmission();
  delay(12);
  return 0;
}

uint8_t DFRobot_WindSpeedWindDirectionRainSensor_IIC::readRegister(uint8_t reg,void* pBuf, size_t size)
{
  if(pBuf == NULL){
    return 0;
  }
  uint8_t * _pBuf = (uint8_t *)pBuf;
  _pWire->beginTransmission(_deviceAddr);
  _pWire->write(reg);
  _pWire->endTransmission();
  _pWire->requestFrom(_deviceAddr, (uint8_t)size );
  for(uint8_t i=0 ;i<size;i++){
    _pBuf[i] = _pWire->read();
  }
  return size;
}