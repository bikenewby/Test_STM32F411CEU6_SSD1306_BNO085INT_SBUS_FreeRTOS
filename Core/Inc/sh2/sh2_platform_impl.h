#ifndef SH2_PLATFORM_IMPL_H
#define SH2_PLATFORM_IMPL_H

#include "sh2_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int sh2_hal_open(sh2_Hal_t *self);
void sh2_hal_close(sh2_Hal_t *self);
int sh2_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
int sh2_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
uint32_t sh2_hal_getTimeUs(sh2_Hal_t *self);

#ifdef __cplusplus
}
#endif

#endif // SH2_PLATFORM_IMPL_H
