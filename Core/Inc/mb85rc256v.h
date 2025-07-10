/*
 * mb85rc256v.h
 *
 *  Created on: Jul 29, 2024
 *      Author: jpgroulx
 */

#include <string.h>
#include "main.h"

#ifndef MB85RC256V_H_
#define MB85RC256V_H_

/* MB85rc Register ----------------------------------------------------------*/
#define MB85rc_ADDRESS 		(0x50<<1)	// mb85rc256v default address
#define MB85rc_EEPROM_SIZE	0x8000		// EEPROM Size 32768 bytes

/* MB85rc External Function -------------------------------------------------*/
void MB85rc_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MB85rc_Bus_Write(uint16_t DevAddr, uint16_t MemAddr, uint8_t *pData, uint16_t Len);
HAL_StatusTypeDef MB85rc_Bus_Read(uint16_t DevAddr, uint16_t MemAddr, uint8_t *pData, uint16_t Len);
HAL_StatusTypeDef MB85rc_ReadByte(uint16_t MemAddr, uint8_t *pData, uint16_t Len);
HAL_StatusTypeDef MB85rc_WriteByte(uint16_t MemAddr, uint8_t *value, uint16_t Len);

#ifdef __cplusplus
}
#endif

#endif /* MB85RC256V_H_ */
