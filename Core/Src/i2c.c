/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
 * @brief iic字节写入函数
 * 
 * @param slave_addr 从机地址
 * @param reg_addr 寄存器地址
 * @param lens 数据长度
 * @param data 数据指针
 * @return int 成功返回0，失败返回1
 */
int IIC_Byte_Write(unsigned char slave_addr,unsigned char reg_addr,unsigned short lens,unsigned char* data){
	return HAL_I2C_Mem_Write(&hi2c1, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, lens, 200);
}

/**
 * @brief IIC字节读取函数
 * 
 * @param slave_addr 从机地址
 * @param reg_addr 寄存器地址
 * @param lens 数据长度
 * @param data 数据指针
 * @return int 成功返回0，失败返回1
 */
int IIC_Byte_Read(unsigned char slave_addr,unsigned char reg_addr,unsigned short lens,unsigned char* data){
	return HAL_I2C_Mem_Read(&hi2c1, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, lens, 200);

}

/**
 * @brief 感应器写函数，并且提供出错尝试机制
 * 
 * @param slave_addr 从机地址
 * @param reg_addr 寄存器地址
 * @param lens 数据长度
 * @param data 数据指针
 * @return int 成功返回0，失败返回1
 */
int Sensor_Write(uint8_t slave_addr,uint8_t reg_addr,unsigned short lens,uint8_t* data){
    int ret=0,retries=0;
    static unsigned int NUM  = 55;
Again:  
    ret = 0;
    ret = IIC_Byte_Write( slave_addr, reg_addr, lens, data);

    if(ret && NUM)
    {
       if( retries++ > 4 )
          return ret;
        
       Soft_Dely(0XFFFFF);
       goto Again;
    } 
    return ret;
    
}

/**
 * @brief 感应器读函数，并且提供出错尝试机制
 * 
 * @param slave_addr 从机地址
 * @param reg_addr 寄存器地址
 * @param lens 数据长度
 * @param data 数据指针
 * @return int 成功返回0，失败返回1
 */
int Sensor_Read(uint8_t slave_addr,uint8_t reg_addr,unsigned short lens,uint8_t* data){
    int ret=0,retries=0;
    static unsigned int NUM  = 55;
Again:  
    ret = 0;
    ret = IIC_Byte_Read( slave_addr, reg_addr, lens, data);

    if(ret && NUM)
    {
        if( retries++ > 4 )
            return ret;
    
        Soft_Dely(NUM);
        goto Again;
    } 
    return ret;
}

/**
 * @brief 软件延时函数
 * 
 * @param num 延时计数num
 */
void Soft_Dely(uint32_t num){
    while(num)
        num--;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
