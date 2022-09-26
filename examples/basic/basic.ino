/*!
 * @file  basic.ino
 * @brief  本示例介绍了DFRobot_ES_RS485库的所有用法
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license    The MIT License (MIT)
 * @author     [fary](feng.yang@dfrobot.com)
 * @version    V1.0
 * @date       2022-07-14
 * @url        https://github.com/DFRobot/DFRobot_ES_RS485
 */
#include "DFRobot_WindSpeedWindDirectionRainSensor.h"

//#define MODE_UART
#ifdef MODE_UART //串口通信
 #if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
   #include "SoftwareSerial.h"
   SoftwareSerial mySerial(/*rx =*/4, /*tx =*/5);
   DFRobot_WindSpeedWindDirectionRainSensor_UART Sensor(/*Stream *=*/&mySerial);
 #else
   DFRobot_WindSpeedWindDirectionRainSensor_UART Sensor(/*Stream *=*/&Serial1);
 #endif
#else //I2C通信
  DFRobot_WindSpeedWindDirectionRainSensor_IIC Sensor(&Wire);
#endif

void setup(void)
{
#ifdef MODE_UART
  #if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
    mySerial.begin(9600);
  #elif defined(ESP32)
    Serial1.begin(9600, SERIAL_8N1, /*rx =*/D3, /*tx =*/D2);
  #else
    Serial1.begin(9600);
  #endif
#endif 
  Serial.begin(9600);
  
  delay(1000);
  while(!Sensor.begin()){
  Serial.println("Sensor init err!!!");
  delay(1000);
  }
  Serial.print("vid:\t");
  Serial.println(Sensor.vid,HEX);
  Serial.print("pid:\t");
  Serial.println(Sensor.pid,HEX);
  Serial.print("Version:\t");
  Serial.println(Sensor.getFirmwareVersion());
  //设置风速计半径，单位为cm
  Sensor.setRadiusAnemometer(9);
  //设置雨量累加值，单位为mm
  Sensor.setRainAccumulatedValue(0.2794);
}

void loop()
{
  //获取实时风向
  Serial.print("Direction:\t");
  Serial.println(Sensor.getWindDirection());
  //获取1分钟内的平均风向(函数参数可选1-5)
  Serial.print("1 minute Direction:\t");
  Serial.println(Sensor.getWindDirection(1));
  //获取实时风速
  Serial.print("Speed:\t");
  Serial.println(Sensor.getWindSpeed());
  //获取一分钟内的平均风速(函数参数可选1-5)
  Serial.print("1 Minute Speed:\t");
  Serial.println(Sensor.getWindSpeed(1));
  //获取系统运行时间的最大风速
  Serial.print("Max Speed:\t");
  Serial.println(Sensor.getMaxWindSpeed());
  //获取一分钟内的最大风速(函数参数可选1-5)
  Serial.print("1 Minute Max Speed:\t");
  Serial.println(Sensor.getMaxWindSpeed(1));
  //获取系统运行时间内的累计雨量
  Serial.print("Rainfall:\t");
  Serial.println(Sensor.getRainfall());
  //获取系统1小时内的累计雨量（函数参数可选1-24）
  Serial.print("1 Hour Rainfall:\t");
  Serial.println(Sensor.getRainfall(1));
  //获取原始数据，1：风向计采集的adc；2：风速传感器的频率，单位 hz；3：雨量的翻斗次数，单位 次
  Serial.print("adc:\t");
  Serial.println(Sensor.getRawdata(1));
  Serial.print("speed Frequency:\t");
  Serial.println(Sensor.getRawdata(2));
  Serial.print("rainfall raw:\t");
  Serial.println(Sensor.getRawdata(3));
  delay(1000);
}