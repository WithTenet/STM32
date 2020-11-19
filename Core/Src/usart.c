/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "stdio.h"
extern uint8_t sendBuf[50];
/* USER CODE END 0 */

UART_HandleTypeDef huart3;

/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOD, USART_TX_Pin|USART_RX_Pin);

  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
 * @brief 非中断方式发送
 * 
 * @param DataToSend 数据指针
 * @param data_num  字符串长度
 */
void USART_NItSend(unsigned char* DataToSend, uint8_t data_num)
{
    HAL_UART_Transmit(&huart3,DataToSend,data_num,0xffff);
}

/**
 * @brief 输出重定向，将printf定向到串口发送
 * 
 * @param ch 发送的字符
 * @param f 文件指针
 * @return int 成功返回字符ascii码
 */

int fputc(int ch,FILE *f)
{
    uint8_t temp[1]={ch};
    HAL_UART_Transmit(&huart3,temp,1,2);       
		return ch;
}

/**
 * @brief 感应器数据发送函数
 * 
 * @param ACCEL_X 加速度计x数据
 * @param ACCEL_Y 加速度计y数据
 * @param ACCEL_Z 加速度计z数据
 * @param GYRO_X 角速度计x数据
 * @param GYRO_Y 角速度计y数据
 * @param GYRO_Z 角速度计z数据
 * @param MAG_X 磁力计x数据
 * @param MAG_Y 磁力计y数据
 * @param MAG_Z 磁力计z数据
 */
void Send_Senser(int16_t ACCEL_X, int16_t ACCEL_Y, int16_t ACCEL_Z, int16_t GYRO_X, int16_t GYRO_Y, int16_t GYRO_Z, int16_t MAG_X, int16_t MAG_Y, int16_t MAG_Z) //鍙戦?佺敤鎴锋暟鎹紝杩欓噷鏈?6涓暟鎹?
{
   uint8_t _cnt = 0;
    uint8_t sum = 0; //以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
    int i;

    sendBuf[_cnt++] = 0xAA; //0xAA为帧头
    sendBuf[_cnt++] = 0x05; //0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
    sendBuf[_cnt++] = 0xAF; //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
    sendBuf[_cnt++] = 0x02; //0x02，表示本帧为传感器原始数据帧
    sendBuf[_cnt++] = 0; //本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

    sendBuf[_cnt++] = BYTE1(ACCEL_X); //将要发送的数据放至发送缓冲区
    sendBuf[_cnt++] = BYTE0(ACCEL_X);

    sendBuf[_cnt++] = BYTE1(ACCEL_Y);
    sendBuf[_cnt++] = BYTE0(ACCEL_Y);

    sendBuf[_cnt++] = BYTE1(ACCEL_Z);
    sendBuf[_cnt++] = BYTE0(ACCEL_Z);

    sendBuf[_cnt++] = BYTE1(GYRO_X);
    sendBuf[_cnt++] = BYTE0(GYRO_X);

    sendBuf[_cnt++] = BYTE1(GYRO_Y);
    sendBuf[_cnt++] = BYTE0(GYRO_Y);

    sendBuf[_cnt++] = BYTE1(GYRO_Z);
    sendBuf[_cnt++] = BYTE0(GYRO_Z);

    sendBuf[_cnt++] = BYTE1(MAG_X);
    sendBuf[_cnt++] = BYTE0(MAG_X);

    sendBuf[_cnt++] = BYTE1(MAG_Y);
    sendBuf[_cnt++] = BYTE0(MAG_Y);

    sendBuf[_cnt++] = BYTE1(MAG_Z);
    sendBuf[_cnt++] = BYTE0(MAG_Z);

    sendBuf[4] = _cnt - 5; //_cnt用来计算数据长度，减5为减去帧开头5个非数据字节

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //将sum校验数据放置最后一字节
    USART_NItSend(sendBuf, _cnt);

}

/**
 * @brief 发送姿态解算三个姿态角给上位机
 * 
 * @param roll 横滚角
 * @param pitch 俯仰角
 * @param yaw 偏航角
 */
void Send_Attitude(float roll, float pitch, float yaw)
{
    uint8_t _cnt = 0;
    uint8_t sum = 0; //以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
    int i;
    int32_t ALT_USE = 0;
    uint8_t FLY_MODEL = 0, ARMED = 0;
    int16_t _temp;

    sendBuf[_cnt++] = 0xAA; //0xAA为帧头
    sendBuf[_cnt++] = 0x05; //0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
    sendBuf[_cnt++] = 0xAF; //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
    sendBuf[_cnt++] = 0x01; //0x01，表示本帧为姿态数据帧
    sendBuf[_cnt++] = 0; //本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

    _temp = (int)(roll * 100);
    sendBuf[_cnt++] = BYTE1(_temp); //将要发送的数据放至发送缓冲区
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = (int)(pitch * 100);
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = (int)(yaw * 100);
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    sendBuf[_cnt++] = BYTE3(ALT_USE); //假数据，为了符合数据帧要求
    sendBuf[_cnt++] = BYTE2(ALT_USE);
    sendBuf[_cnt++] = BYTE1(ALT_USE);
    sendBuf[_cnt++] = BYTE0(ALT_USE);

    sendBuf[_cnt++] = FLY_MODEL;
    sendBuf[_cnt++] = ARMED;

    sendBuf[4] = _cnt - 5; //_cnt用来计算数据长度，减5为减去帧开头5个非数据字节

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //将sum校验数据放置最后一字节

    USART_NItSend(sendBuf, _cnt);

}

/**
 * @brief 发送遥控器以及电机PWM信号给上位机
 * 
 * @param THR 油门信号
 * @param YAW 偏航信号
 * @param ROLL 横滚信号
 * @param PITCH 俯仰信号
 * @param motor1 电机1PWM
 * @param motor2 电机2PWM
 * @param motor3 电机3PWM
 * @param motor4 电机4PWM
 */
void Send_RCData_Motor(int16_t THR, int16_t YAW, int16_t ROLL, int16_t PITCH, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint8_t _cnt = 0;
    uint8_t sum = 0; //以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
    int i;
    int16_t AUX = 0;

    sendBuf[_cnt++] = 0xAA; //0xAA为帧头
    sendBuf[_cnt++] = 0x05; //0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
    sendBuf[_cnt++] = 0xAF; //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
    sendBuf[_cnt++] = 0x03; //0x03，表示本帧为接收机、电机速度数据帧
    sendBuf[_cnt++] = 0; //本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

    sendBuf[_cnt++] = BYTE1(THR); //将要发送的数据放至发送缓冲区
    sendBuf[_cnt++] = BYTE0(THR);

    sendBuf[_cnt++] = BYTE1(YAW);
    sendBuf[_cnt++] = BYTE0(YAW);

    sendBuf[_cnt++] = BYTE1(ROLL);
    sendBuf[_cnt++] = BYTE0(ROLL);

    sendBuf[_cnt++] = BYTE1(PITCH);
    sendBuf[_cnt++] = BYTE0(PITCH);

    sendBuf[_cnt++] = BYTE1(motor1);
    sendBuf[_cnt++] = BYTE0(motor1);

    sendBuf[_cnt++] = BYTE1(motor2);
    sendBuf[_cnt++] = BYTE0(motor2);

    sendBuf[_cnt++] = BYTE1(motor3);
    sendBuf[_cnt++] = BYTE0(motor3);

    sendBuf[_cnt++] = BYTE1(motor4);
    sendBuf[_cnt++] = BYTE0(motor4);

    sendBuf[_cnt++] = BYTE1(AUX);
    sendBuf[_cnt++] = BYTE0(AUX);

    sendBuf[_cnt++] = BYTE1(AUX);
    sendBuf[_cnt++] = BYTE0(AUX);

    sendBuf[4] = _cnt - 5; //_cnt用来计算数据长度，减5为减去帧开头5个非数据字节

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //将sum校验数据放置最后一字节
		
    USART_NItSend(sendBuf, _cnt);
}

/**
 * @brief 发送字符串给上位机
 * 
 * @param str 字符串指针
 */
void SendStr(const char* str)
{
    uint8_t _cnt = 0;
    uint8_t i = 0;
    uint8_t sum = 0;

    sendBuf[_cnt++] = 0xAA;
    sendBuf[_cnt++] = 0x05;
    sendBuf[_cnt++] = 0xAF;
    sendBuf[_cnt++] = 0xA0;
    sendBuf[_cnt++] = 0;
    while (*(str + i) != '\0') {
        sendBuf[_cnt++] = *(str + i);
        i++;
        if (_cnt > 50)
            break;
    }

    sendBuf[4] = _cnt - 5;

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum;

    USART_NItSend(sendBuf, _cnt);
}

/**
 * @brief 发送字节数据给上位机
 * 
 * @param frame 用户自定义字节
 * @param p 数据指针
 */
void SendByte(uint8_t frame, uint8_t* p)
{
    uint8_t _cnt = 0;
    uint8_t sum = 0; //以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
    int i;

    sendBuf[_cnt++] = 0xAA; //0xAA为帧头
    sendBuf[_cnt++] = 0x05; //0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
    sendBuf[_cnt++] = 0xAF; //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
    sendBuf[_cnt++] = frame; //用户自定义数据帧
    sendBuf[_cnt++] = 0; //本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

    sendBuf[_cnt++] = *p; //将要发送的数据放至发送缓冲区

    sendBuf[4] = _cnt - 5; //_cnt用来计算数据长度，减5为减去帧开头5个非数据字节

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //将sum校验数据放置最后一字节

    USART_NItSend(sendBuf, _cnt);
}

/**
 * @brief 发送半字给上位机
 * 
 * @param frame 用户自定义功能字节
 * @param p 数据指针
 */
void SendHalfWord(uint8_t frame, uint16_t* p)
{
   uint8_t _cnt = 0;
    uint8_t sum = 0; //以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
    int i;

    sendBuf[_cnt++] = 0xAA; //0xAA为帧头
    sendBuf[_cnt++] = 0x05; //0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
    sendBuf[_cnt++] = 0xAF; //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
    sendBuf[_cnt++] = frame; //用户自定义数据帧
    sendBuf[_cnt++] = 0; //本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

    sendBuf[_cnt++] = BYTE1(*p); //将要发送的数据放至发送缓冲区
    sendBuf[_cnt++] = BYTE0(*p);

    sendBuf[4] = _cnt - 5; //_cnt用来计算数据长度，减5为减去帧开头5个非数据字节

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //将sum校验数据放置最后一字节

    USART_NItSend(sendBuf, _cnt);
}

/**
 * @brief 发送字数据给上位机
 * @param frame 用户自定义功能字节
 * @param p 数据指针
 */
void SendWord(uint8_t frame, uint32_t* p)
{
   uint8_t _cnt = 0;
    uint8_t sum = 0; //以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
    int i;

    sendBuf[_cnt++] = 0xAA; //0xAA为帧头
    sendBuf[_cnt++] = 0x05; //0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
    sendBuf[_cnt++] = 0xAF; //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
    sendBuf[_cnt++] = frame; //用户自定义数据帧
    sendBuf[_cnt++] = 0; //本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

    sendBuf[_cnt++] = BYTE3(*p); //将要发送的数据放至发送缓冲区
    sendBuf[_cnt++] = BYTE2(*p);
    sendBuf[_cnt++] = BYTE1(*p);
    sendBuf[_cnt++] = BYTE0(*p);

    sendBuf[4] = _cnt - 5; //_cnt用来计算数据长度，减5为减去帧开头5个非数据字节

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //将sum校验数据放置最后一字节

    USART_NItSend(sendBuf, _cnt);
}

/**
 * @brief 发哦那个5浮点数给上位机
 * 
 * @param frame 用户自定义功能字
 * @param f1 浮点数1
 * @param f2 浮点数2
 * @param f3 浮点数3
 * @param f4 浮点数4
 * @param f5 浮点数5
 */
void Send_5_float(uint8_t frame,float f1,float f2,float f3,float f4,float f5){
    uint8_t _cnt = 0;
    uint8_t sum = 0; //以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
    int i;
    int16_t _temp;

    sendBuf[_cnt++] = 0xAA; //0xAA为帧头
    sendBuf[_cnt++] = 0x05; //0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
    sendBuf[_cnt++] = 0xAF; //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
    sendBuf[_cnt++] = frame; //用户自定义帧
    sendBuf[_cnt++] = 0; //本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

    _temp = f1;
    sendBuf[_cnt++] = BYTE1(_temp); //将要发送的数据放至发送缓冲区
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = f2;
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = f3;
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = f4;
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = f5;
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    sendBuf[4] = _cnt - 5; //_cnt用来计算数据长度，减5为减去帧开头5个非数据字节

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //将sum校验数据放置最后一字节

    USART_NItSend(sendBuf, _cnt);
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
