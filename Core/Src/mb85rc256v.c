/*
 * mb85rc256v.c
 *
 *  Created on: Jul 29, 2024
 *      Author: johng
 */

#include "mb85rc256v.h"
#include "stdio.h"

I2C_HandleTypeDef *i2c;

/**
  * @brief  Initialize local I2C handle
  */
void MB85rc_Init(I2C_HandleTypeDef *hi2c) {
	i2c = hi2c;
}

/**
  * @brief  I2C Bus Write 16bit
  * @retval HAL_StatusTypeDef
  * @param  DevAddr		Target device address
  * @param  memAddr  	Internal memory address
  * @param  pData 		Pointer to data buffer
  * @param  Len  		Amount of data to be Write
  */
HAL_StatusTypeDef MB85rc_Bus_Write(uint16_t DevAddr, uint16_t memAddr, uint8_t *pData, uint16_t Len)
{
	HAL_StatusTypeDef halStatus = HAL_OK;

	halStatus = HAL_I2C_Mem_Write(i2c, DevAddr, memAddr, I2C_MEMADD_SIZE_16BIT, pData, Len, HAL_MAX_DELAY);

    return halStatus;
}

/**
  * @brief  I2C Bus Read 16bits
  * @retval HAL_StatusTypeDef
  * @param  DevAddr		Target device address
  * @param  memAddr  	Internal memory address
  * @param  pData 		Pointer to data buffer
  * @param  Len  		Amount of data to be read
  */
HAL_StatusTypeDef MB85rc_Bus_Read(uint16_t DevAddr, uint16_t memAddr, uint8_t *pData, uint16_t Len)
{
	HAL_StatusTypeDef halStatus = HAL_OK;

	halStatus = HAL_I2C_Mem_Read(i2c, DevAddr, memAddr, I2C_MEMADD_SIZE_16BIT, pData, Len, HAL_MAX_DELAY);

    return halStatus;
}

/**
  * @brief  Sequential read from selected memory address with number of byte
  * @retval HAL_StatusTypeDef
  * @param  memAddr	Memory address to start read from
  * @param  pData	Pointer to data
  * @param  Len		Number of byte to read
  */
HAL_StatusTypeDef MB85rc_ReadByte(uint16_t memAddr, uint8_t *pData, uint16_t Len)
{
	HAL_StatusTypeDef halStatus = HAL_OK;

	halStatus = MB85rc_Bus_Read(MB85rc_ADDRESS, memAddr, pData, Len);

	return halStatus;
}

/**
  * @brief  Write data into EEPROM
  * @retval HAL_StatusTypeDef
  * @param  memAddr	Memory address to start write from
  * @param  pData	Pointer to data
  * @param  Len		Number of byte to write
  */
HAL_StatusTypeDef MB85rc_WriteByte(uint16_t memAddr, uint8_t *value, uint16_t Len)
{
	HAL_StatusTypeDef halStatus = HAL_OK;

	halStatus = MB85rc_Bus_Write(MB85rc_ADDRESS, memAddr, value, Len);

	return halStatus;
}
