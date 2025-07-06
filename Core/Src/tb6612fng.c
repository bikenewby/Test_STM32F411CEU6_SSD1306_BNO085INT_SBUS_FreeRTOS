#include "tb6612fng.h"

// Extern timer handles from main.c
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

// Channel configuration table (update pins/timers as per CubeMX and main.h)
static TB6612FNG_Channel tb6612fng_channels[MOTOR_COUNT] = {
    // MOTOR_1A: PA0, PA1, TIM1_CH1 (PA8)
    {GPIOA, GPIO_PIN_0, GPIOA, GPIO_PIN_1, &htim1, TIM_CHANNEL_1},
    // MOTOR_1B: PB5, PA15, TIM1_CH2 (PA9)
    {GPIOB, GPIO_PIN_5, GPIOA, GPIO_PIN_15, &htim1, TIM_CHANNEL_2},
    // MOTOR_2A: PB0, PB1, TIM1_CH4 (PA11)
    {GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_1, &htim1, TIM_CHANNEL_4},
    // MOTOR_2B: PB14, PB15, TIM3_CH2 (PA7)
    {GPIOB, GPIO_PIN_14, GPIOB, GPIO_PIN_15, &htim3, TIM_CHANNEL_2}
};

void TB6612FNG_Init(void) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        HAL_GPIO_WritePin(tb6612fng_channels[i].IN1_Port, tb6612fng_channels[i].IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(tb6612fng_channels[i].IN2_Port, tb6612fng_channels[i].IN2_Pin, GPIO_PIN_RESET);
        HAL_TIM_PWM_Start(tb6612fng_channels[i].PWM_Timer, tb6612fng_channels[i].PWM_Channel);
    }
    // Set STBY pins high to enable drivers (if used)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // TB6612 #1 STBY
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // TB6612 #2 STBY
}

void TB6612FNG_Set(TB6612FNG_Channel* ch, TB6612FNG_Dir dir, uint16_t speed) {
    if (!ch) return;
    // Get timer period
    uint32_t period = ch->PWM_Timer->Instance->ARR;
    if (speed > period) speed = period; // Clamp speed to period

    switch (dir) {
        case TB6612FNG_FORWARD:
            HAL_GPIO_WritePin(ch->IN1_Port, ch->IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ch->IN2_Port, ch->IN2_Pin, GPIO_PIN_RESET);
            break;
        case TB6612FNG_BACKWARD:
            HAL_GPIO_WritePin(ch->IN1_Port, ch->IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ch->IN2_Port, ch->IN2_Pin, GPIO_PIN_SET);
            break;
        case TB6612FNG_BRAKE:
            HAL_GPIO_WritePin(ch->IN1_Port, ch->IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ch->IN2_Port, ch->IN2_Pin, GPIO_PIN_SET);
            break;
        case TB6612FNG_STOP:
        default:
            HAL_GPIO_WritePin(ch->IN1_Port, ch->IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ch->IN2_Port, ch->IN2_Pin, GPIO_PIN_RESET);
            break;
    }
    __HAL_TIM_SET_COMPARE(ch->PWM_Timer, ch->PWM_Channel, speed);
}


void tb6612fng_drive(TB6612FNG_Motor motor, TB6612FNG_Dir dir, uint16_t speed) {
    if (motor < MOTOR_COUNT)
        TB6612FNG_Set(&tb6612fng_channels[motor], dir, speed);
}

void tb6612fng_stop(TB6612FNG_Motor motor) {
    if (motor < MOTOR_COUNT)
        TB6612FNG_Set(&tb6612fng_channels[motor], TB6612FNG_STOP, 0);
}

void tb6612fng_brake(TB6612FNG_Motor motor) {
    if (motor < MOTOR_COUNT)
        TB6612FNG_Set(&tb6612fng_channels[motor], TB6612FNG_BRAKE, 0);
}
