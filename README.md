# 四轴飞行器(HAL版本)

## 1.项目简介：

​	本项目中使用了蓝牙与上位机通信，IIC采集姿态传感器数据，使用九轴数据进行互补滤波完成姿态解算，并且移植了uCOS操作系统完成多任务系统。

## 2.芯片配置

|  设备  |  协议 |   引脚 |
| :---- | :---- | :---- |
| 蓝牙 | USART | USART3-TX:PD9  RX:PD8 |
| GY-86 | IIC | IIC1-SCL:PB6  SDA:PB7 |
| 电调 | PWM | TIM8-CH1:PE9  CH2:PE11<br />CH3:PE13  CH4:PE14 |
| 接收机 | PWM | TIM1-CH1:PC6  CH2:PC7<br />CH3:PC8  CH4:PC9 |

## 3.开发工具

 [^keil5] & [^STM32CubeMX]

## 4.开始

1.打开MDK-ARM中的工程文件

2.编译下载到芯片

注：项目根目录中包含自动上传脚本

gitinit.bat:自动上传前准备

upload.bat:自动上传执行













[^STM32CubeMX]: 项目中使用STM32CubeMX生成项目主题框架结构



[^keil5]: keil5进行移植操作系统以及上层应用开发



