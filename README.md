# STM32F407 Peripheral Demo
These Programs are a part of personal hobby projects intended to act as a demonstration of skill set.
All programs here are developed using HAL libraries in conjunction with STM32CubeMx Code Generator.

Program List Summary
1) An ambient light detector using LDR and LED interfaced to STM32F407VGTx. 
  This program demonstrates the use of Analog to Digital Converter built in STM32F407 Discovery board to read converted LDR sensor data in   polling mode. 
2)An ambient light detector using LDR, LED and push button interfaced to STM32F407VGTx. 
  This program demonstrates the use of Analog to Digital Converter built in STM32F407 Discovery board to read converted LDR sensor data in   interrupt mode. A push button triggers the A/D conversion.This program also illustrates the use of interrupts with GPIO
3)A Program to read temperature and humididty from Bosch BMP180 I2C based Sensor.
  This program shows how to use HAL Libraries to communicate with i2c based sensors
