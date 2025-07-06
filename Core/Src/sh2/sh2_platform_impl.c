#include "sh2_platform_impl.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

extern I2C_HandleTypeDef hi2c1;
extern osMutexId_t I2C1MutexHandle; // Use the same mutex for accessing I2C1
#define BNO085_I2C_ADDR   (0x4A << 1)

int sh2_hal_open(sh2_Hal_t *self)   { return 0; }
void sh2_hal_close(sh2_Hal_t *self) { }

int sh2_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    int ret = 0;
    osMutexAcquire(I2C1MutexHandle, osWaitForever);
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&hi2c1, BNO085_I2C_ADDR, pBuffer, len, 1000);
    osMutexRelease(I2C1MutexHandle);
    if (status == HAL_OK) {
        if (t_us) *t_us = HAL_GetTick() * 1000;
        ret = len;
    }
    return ret;
}

int sh2_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    int ret = 0;
    osMutexAcquire(I2C1MutexHandle, osWaitForever);
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, BNO085_I2C_ADDR, pBuffer, len, 1000);
    osMutexRelease(I2C1MutexHandle);
    if (status == HAL_OK) ret = len;
    return ret;
}

uint32_t sh2_hal_getTimeUs(sh2_Hal_t *self) {
    return HAL_GetTick() * 1000;
}
