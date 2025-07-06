#ifndef TB6612FNG_H
#define TB6612FNG_H

#include "stm32f4xx_hal.h"

// Motor channel identifiers
typedef enum {
    MOTOR_1A = 0,
    MOTOR_1B,
    MOTOR_2A,
    MOTOR_2B,
    MOTOR_COUNT
} TB6612FNG_Motor;

// Motor direction
typedef enum {
    TB6612FNG_STOP = 0,
    TB6612FNG_FORWARD,
    TB6612FNG_BACKWARD,
    TB6612FNG_BRAKE
} TB6612FNG_Dir;

// Motor channel configuration
typedef struct {
    GPIO_TypeDef* IN1_Port;
    uint16_t      IN1_Pin;
    GPIO_TypeDef* IN2_Port;
    uint16_t      IN2_Pin;
    TIM_HandleTypeDef* PWM_Timer;
    uint32_t      PWM_Channel;
} TB6612FNG_Channel;

// Initialization: call once in main.c after timers are initialized
void TB6612FNG_Init(void);

// Direct set function (low-level)
void TB6612FNG_Set(TB6612FNG_Channel* ch, TB6612FNG_Dir dir, uint16_t speed);

// High-level drive/stop/brake functions
void tb6612fng_drive(TB6612FNG_Motor motor, TB6612FNG_Dir dir, uint16_t speed);
void tb6612fng_stop(TB6612FNG_Motor motor);
void tb6612fng_brake(TB6612FNG_Motor motor);

#endif // TB6612FNG_H
