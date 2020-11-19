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
 * @brief ���жϷ�ʽ����
 * 
 * @param DataToSend ����ָ��
 * @param data_num  �ַ�������
 */
void USART_NItSend(unsigned char* DataToSend, uint8_t data_num)
{
    HAL_UART_Transmit(&huart3,DataToSend,data_num,0xffff);
}

/**
 * @brief ����ض��򣬽�printf���򵽴��ڷ���
 * 
 * @param ch ���͵��ַ�
 * @param f �ļ�ָ��
 * @return int �ɹ������ַ�ascii��
 */

int fputc(int ch,FILE *f)
{
    uint8_t temp[1]={ch};
    HAL_UART_Transmit(&huart3,temp,1,2);       
		return ch;
}

/**
 * @brief ��Ӧ�����ݷ��ͺ���
 * 
 * @param ACCEL_X ���ٶȼ�x����
 * @param ACCEL_Y ���ٶȼ�y����
 * @param ACCEL_Z ���ٶȼ�z����
 * @param GYRO_X ���ٶȼ�x����
 * @param GYRO_Y ���ٶȼ�y����
 * @param GYRO_Z ���ٶȼ�z����
 * @param MAG_X ������x����
 * @param MAG_Y ������y����
 * @param MAG_Z ������z����
 */
void Send_Senser(int16_t ACCEL_X, int16_t ACCEL_Y, int16_t ACCEL_Z, int16_t GYRO_X, int16_t GYRO_Y, int16_t GYRO_Z, int16_t MAG_X, int16_t MAG_Y, int16_t MAG_Z) //发�?�用户数据，这里�?6个数�?
{
   uint8_t _cnt = 0;
    uint8_t sum = 0; //����Ϊ����sumУ���ֽڣ���0xAAҲ�������ֽڣ�һֱ��sum�ֽ�ǰһ�ֽ�
    int i;

    sendBuf[_cnt++] = 0xAA; //0xAAΪ֡ͷ
    sendBuf[_cnt++] = 0x05; //0x05Ϊ���ݷ���Դ��������ο�����Э�飬���ֽ��û������������
    sendBuf[_cnt++] = 0xAF; //0xAFΪ����Ŀ�ĵأ�AF��ʾ��λ����������ο�����Э��
    sendBuf[_cnt++] = 0x02; //0x02����ʾ��֡Ϊ������ԭʼ����֡
    sendBuf[_cnt++] = 0; //���ֽڱ�ʾ���ݳ��ȣ�������=0����������ٸ�ֵ�������Ͳ����˹����㳤����

    sendBuf[_cnt++] = BYTE1(ACCEL_X); //��Ҫ���͵����ݷ������ͻ�����
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

    sendBuf[4] = _cnt - 5; //_cnt�����������ݳ��ȣ���5Ϊ��ȥ֡��ͷ5���������ֽ�

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //��sumУ�����ݷ������һ�ֽ�
    USART_NItSend(sendBuf, _cnt);

}

/**
 * @brief ������̬����������̬�Ǹ���λ��
 * 
 * @param roll �����
 * @param pitch ������
 * @param yaw ƫ����
 */
void Send_Attitude(float roll, float pitch, float yaw)
{
    uint8_t _cnt = 0;
    uint8_t sum = 0; //����Ϊ����sumУ���ֽڣ���0xAAҲ�������ֽڣ�һֱ��sum�ֽ�ǰһ�ֽ�
    int i;
    int32_t ALT_USE = 0;
    uint8_t FLY_MODEL = 0, ARMED = 0;
    int16_t _temp;

    sendBuf[_cnt++] = 0xAA; //0xAAΪ֡ͷ
    sendBuf[_cnt++] = 0x05; //0x05Ϊ���ݷ���Դ��������ο�����Э�飬���ֽ��û������������
    sendBuf[_cnt++] = 0xAF; //0xAFΪ����Ŀ�ĵأ�AF��ʾ��λ����������ο�����Э��
    sendBuf[_cnt++] = 0x01; //0x01����ʾ��֡Ϊ��̬����֡
    sendBuf[_cnt++] = 0; //���ֽڱ�ʾ���ݳ��ȣ�������=0����������ٸ�ֵ�������Ͳ����˹����㳤����

    _temp = (int)(roll * 100);
    sendBuf[_cnt++] = BYTE1(_temp); //��Ҫ���͵����ݷ������ͻ�����
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = (int)(pitch * 100);
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = (int)(yaw * 100);
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    sendBuf[_cnt++] = BYTE3(ALT_USE); //�����ݣ�Ϊ�˷�������֡Ҫ��
    sendBuf[_cnt++] = BYTE2(ALT_USE);
    sendBuf[_cnt++] = BYTE1(ALT_USE);
    sendBuf[_cnt++] = BYTE0(ALT_USE);

    sendBuf[_cnt++] = FLY_MODEL;
    sendBuf[_cnt++] = ARMED;

    sendBuf[4] = _cnt - 5; //_cnt�����������ݳ��ȣ���5Ϊ��ȥ֡��ͷ5���������ֽ�

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //��sumУ�����ݷ������һ�ֽ�

    USART_NItSend(sendBuf, _cnt);

}

/**
 * @brief ����ң�����Լ����PWM�źŸ���λ��
 * 
 * @param THR �����ź�
 * @param YAW ƫ���ź�
 * @param ROLL ����ź�
 * @param PITCH �����ź�
 * @param motor1 ���1PWM
 * @param motor2 ���2PWM
 * @param motor3 ���3PWM
 * @param motor4 ���4PWM
 */
void Send_RCData_Motor(int16_t THR, int16_t YAW, int16_t ROLL, int16_t PITCH, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint8_t _cnt = 0;
    uint8_t sum = 0; //����Ϊ����sumУ���ֽڣ���0xAAҲ�������ֽڣ�һֱ��sum�ֽ�ǰһ�ֽ�
    int i;
    int16_t AUX = 0;

    sendBuf[_cnt++] = 0xAA; //0xAAΪ֡ͷ
    sendBuf[_cnt++] = 0x05; //0x05Ϊ���ݷ���Դ��������ο�����Э�飬���ֽ��û������������
    sendBuf[_cnt++] = 0xAF; //0xAFΪ����Ŀ�ĵأ�AF��ʾ��λ����������ο�����Э��
    sendBuf[_cnt++] = 0x03; //0x03����ʾ��֡Ϊ���ջ�������ٶ�����֡
    sendBuf[_cnt++] = 0; //���ֽڱ�ʾ���ݳ��ȣ�������=0����������ٸ�ֵ�������Ͳ����˹����㳤����

    sendBuf[_cnt++] = BYTE1(THR); //��Ҫ���͵����ݷ������ͻ�����
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

    sendBuf[4] = _cnt - 5; //_cnt�����������ݳ��ȣ���5Ϊ��ȥ֡��ͷ5���������ֽ�

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //��sumУ�����ݷ������һ�ֽ�
		
    USART_NItSend(sendBuf, _cnt);
}

/**
 * @brief �����ַ�������λ��
 * 
 * @param str �ַ���ָ��
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
 * @brief �����ֽ����ݸ���λ��
 * 
 * @param frame �û��Զ����ֽ�
 * @param p ����ָ��
 */
void SendByte(uint8_t frame, uint8_t* p)
{
    uint8_t _cnt = 0;
    uint8_t sum = 0; //����Ϊ����sumУ���ֽڣ���0xAAҲ�������ֽڣ�һֱ��sum�ֽ�ǰһ�ֽ�
    int i;

    sendBuf[_cnt++] = 0xAA; //0xAAΪ֡ͷ
    sendBuf[_cnt++] = 0x05; //0x05Ϊ���ݷ���Դ��������ο�����Э�飬���ֽ��û������������
    sendBuf[_cnt++] = 0xAF; //0xAFΪ����Ŀ�ĵأ�AF��ʾ��λ����������ο�����Э��
    sendBuf[_cnt++] = frame; //�û��Զ�������֡
    sendBuf[_cnt++] = 0; //���ֽڱ�ʾ���ݳ��ȣ�������=0����������ٸ�ֵ�������Ͳ����˹����㳤����

    sendBuf[_cnt++] = *p; //��Ҫ���͵����ݷ������ͻ�����

    sendBuf[4] = _cnt - 5; //_cnt�����������ݳ��ȣ���5Ϊ��ȥ֡��ͷ5���������ֽ�

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //��sumУ�����ݷ������һ�ֽ�

    USART_NItSend(sendBuf, _cnt);
}

/**
 * @brief ���Ͱ��ָ���λ��
 * 
 * @param frame �û��Զ��幦���ֽ�
 * @param p ����ָ��
 */
void SendHalfWord(uint8_t frame, uint16_t* p)
{
   uint8_t _cnt = 0;
    uint8_t sum = 0; //����Ϊ����sumУ���ֽڣ���0xAAҲ�������ֽڣ�һֱ��sum�ֽ�ǰһ�ֽ�
    int i;

    sendBuf[_cnt++] = 0xAA; //0xAAΪ֡ͷ
    sendBuf[_cnt++] = 0x05; //0x05Ϊ���ݷ���Դ��������ο�����Э�飬���ֽ��û������������
    sendBuf[_cnt++] = 0xAF; //0xAFΪ����Ŀ�ĵأ�AF��ʾ��λ����������ο�����Э��
    sendBuf[_cnt++] = frame; //�û��Զ�������֡
    sendBuf[_cnt++] = 0; //���ֽڱ�ʾ���ݳ��ȣ�������=0����������ٸ�ֵ�������Ͳ����˹����㳤����

    sendBuf[_cnt++] = BYTE1(*p); //��Ҫ���͵����ݷ������ͻ�����
    sendBuf[_cnt++] = BYTE0(*p);

    sendBuf[4] = _cnt - 5; //_cnt�����������ݳ��ȣ���5Ϊ��ȥ֡��ͷ5���������ֽ�

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //��sumУ�����ݷ������һ�ֽ�

    USART_NItSend(sendBuf, _cnt);
}

/**
 * @brief ���������ݸ���λ��
 * @param frame �û��Զ��幦���ֽ�
 * @param p ����ָ��
 */
void SendWord(uint8_t frame, uint32_t* p)
{
   uint8_t _cnt = 0;
    uint8_t sum = 0; //����Ϊ����sumУ���ֽڣ���0xAAҲ�������ֽڣ�һֱ��sum�ֽ�ǰһ�ֽ�
    int i;

    sendBuf[_cnt++] = 0xAA; //0xAAΪ֡ͷ
    sendBuf[_cnt++] = 0x05; //0x05Ϊ���ݷ���Դ��������ο�����Э�飬���ֽ��û������������
    sendBuf[_cnt++] = 0xAF; //0xAFΪ����Ŀ�ĵأ�AF��ʾ��λ����������ο�����Э��
    sendBuf[_cnt++] = frame; //�û��Զ�������֡
    sendBuf[_cnt++] = 0; //���ֽڱ�ʾ���ݳ��ȣ�������=0����������ٸ�ֵ�������Ͳ����˹����㳤����

    sendBuf[_cnt++] = BYTE3(*p); //��Ҫ���͵����ݷ������ͻ�����
    sendBuf[_cnt++] = BYTE2(*p);
    sendBuf[_cnt++] = BYTE1(*p);
    sendBuf[_cnt++] = BYTE0(*p);

    sendBuf[4] = _cnt - 5; //_cnt�����������ݳ��ȣ���5Ϊ��ȥ֡��ͷ5���������ֽ�

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //��sumУ�����ݷ������һ�ֽ�

    USART_NItSend(sendBuf, _cnt);
}

/**
 * @brief ��Ŷ�Ǹ�5����������λ��
 * 
 * @param frame �û��Զ��幦����
 * @param f1 ������1
 * @param f2 ������2
 * @param f3 ������3
 * @param f4 ������4
 * @param f5 ������5
 */
void Send_5_float(uint8_t frame,float f1,float f2,float f3,float f4,float f5){
    uint8_t _cnt = 0;
    uint8_t sum = 0; //����Ϊ����sumУ���ֽڣ���0xAAҲ�������ֽڣ�һֱ��sum�ֽ�ǰһ�ֽ�
    int i;
    int16_t _temp;

    sendBuf[_cnt++] = 0xAA; //0xAAΪ֡ͷ
    sendBuf[_cnt++] = 0x05; //0x05Ϊ���ݷ���Դ��������ο�����Э�飬���ֽ��û������������
    sendBuf[_cnt++] = 0xAF; //0xAFΪ����Ŀ�ĵأ�AF��ʾ��λ����������ο�����Э��
    sendBuf[_cnt++] = frame; //�û��Զ���֡
    sendBuf[_cnt++] = 0; //���ֽڱ�ʾ���ݳ��ȣ�������=0����������ٸ�ֵ�������Ͳ����˹����㳤����

    _temp = f1;
    sendBuf[_cnt++] = BYTE1(_temp); //��Ҫ���͵����ݷ������ͻ�����
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

    sendBuf[4] = _cnt - 5; //_cnt�����������ݳ��ȣ���5Ϊ��ȥ֡��ͷ5���������ֽ�

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //��sumУ�����ݷ������һ�ֽ�

    USART_NItSend(sendBuf, _cnt);
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
