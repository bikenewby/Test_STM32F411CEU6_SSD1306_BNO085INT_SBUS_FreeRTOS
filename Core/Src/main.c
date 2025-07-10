/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "sh2/sh2.h"
#include "sh2/sh2_SensorValue.h"
#include "sh2/sh2_platform_impl.h"
#include "tb6612fng.h"
#include "ee.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_DEBOUNCE_MS 150
#define BUTTON_DEBOUNCE_MS 400
#define CLK_SW_DEBOUNCE_MS 30
#define SBUS_FRAME_SIZE 25

// Robot operation modes
typedef enum {
	MODE_DISPLAY_SENSORS = 0, MODE_MOVEMENT = 1, // Robot movement w/o heading lock
	MODE_MOVEMENT_HL = 2,     // Robot movement with heading lock
	MODE_COUNT = 3
} robot_mode_t;

// Robot states
typedef enum {
	STATE_IDLE = 0, STATE_RUNNING = 1
} robot_state_t;

// SBUS Channel 7 mode control with dead zones
typedef enum {
	CH7_POSITION_MIN = 0,
	CH7_POSITION_MID = 1,
	CH7_POSITION_MAX = 2,
	CH7_POSITION_UNKNOWN = 3
} ch7_position_t;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CompassTask */
osThreadId_t CompassTaskHandle;
const osThreadAttr_t CompassTask_attributes = {
  .name = "CompassTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for EncoderTask */
osThreadId_t EncoderTaskHandle;
const osThreadAttr_t EncoderTask_attributes = {
  .name = "EncoderTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CompassMutex */
osMutexId_t CompassMutexHandle;
const osMutexAttr_t CompassMutex_attributes = {
  .name = "CompassMutex"
};
/* Definitions for I2C1Mutex */
osMutexId_t I2C1MutexHandle;
const osMutexAttr_t I2C1Mutex_attributes = {
  .name = "I2C1Mutex"
};
/* Definitions for EncoderMutex */
osMutexId_t EncoderMutexHandle;
const osMutexAttr_t EncoderMutex_attributes = {
  .name = "EncoderMutex"
};
/* Definitions for UARTMutex */
osMutexId_t UARTMutexHandle;
const osMutexAttr_t UARTMutex_attributes = {
  .name = "UARTMutex"
};
/* Definitions for ModeMutex */
osMutexId_t ModeMutexHandle;
const osMutexAttr_t ModeMutex_attributes = {
  .name = "ModeMutex"
};
/* Definitions for EEPROMMutex */
osMutexId_t EEPROMMutexHandle;
const osMutexAttr_t EEPROMMutex_attributes = {
  .name = "EEPROMMutex"
};
/* USER CODE BEGIN PV */
// USART
uint8_t sbus_rx_buffer[SBUS_FRAME_SIZE];
volatile uint16_t sbus_channels[16] = { 0 };
volatile uint32_t uart2_error_code = 0;
volatile uint8_t uart2_error_flag = 0;
// SBUS signal monitoring
volatile uint32_t sbus_last_valid_time = 0;
volatile uint8_t sbus_signal_valid = 0;
#define SBUS_TIMEOUT_MS 100  // Stop motors if no valid SBUS for 100ms
// SBUS recovery variables
volatile uint8_t sbus_recovery_attempts = 0;
volatile uint32_t sbus_last_error_time = 0;
volatile uint8_t sbus_recovery_requested = 0;
#define SBUS_MAX_RECOVERY_ATTEMPTS 5
#define SBUS_ERROR_COOLDOWN_MS 1000
// Compass variables
volatile uint8_t bno085_int_flag = 0;
volatile float compass_heading_deg = 0.0f;
// Encoder variables
volatile int32_t encoder_count = 0;
volatile uint8_t encoder_button_pressed = 0;
volatile int32_t encoder_sw_count = 0;
uint32_t last_clk_tick = 0;
uint32_t last_sw_tick = 0;

// Mode control variables
volatile robot_mode_t current_mode = MODE_DISPLAY_SENSORS;
volatile robot_mode_t selected_mode = MODE_DISPLAY_SENSORS;
volatile robot_state_t robot_state = STATE_IDLE;
volatile uint8_t mode_change_request = 0;
volatile uint8_t state_toggle_request = 0;

// Motor control variables for 3-wheel omnidirectional robot
volatile int16_t joystick_x = 0;          // Left/Right (-100 to +100)
volatile int16_t joystick_y = 0;          // Forward/Backward (-100 to +100)
volatile int16_t joystick_rotation = 0;   // Rotation (-100 to +100)
volatile int16_t motor_left_front = 0;    // Motor 1B (Left Front)
volatile int16_t motor_right_front = 0;   // Motor 1A (Right Front)
volatile int16_t motor_back = 0;          // Motor 2A (Back)

// Heading lock and PID control variables
volatile uint8_t heading_lock_enabled = 0;
volatile float locked_heading = 0.0f;
volatile float heading_error = 0.0f;
volatile float heading_pid_output = 0.0f;

// PID parameters for heading control
#define PID_KP 0.7f    // Reduced for smoother response
#define PID_KI 0.02f   // Much lower to prevent windup
#define PID_KD 0.1f    // Lower to reduce noise sensitivity
//#define PID_MAX_OUTPUT 15.0f  // Limited to 15% for gentle correction
#define PID_MAX_OUTPUT 18.0f  // Limited to 18% for gentle correction
#define PID_INTEGRAL_MAX 20.0f  // Reduced anti-windup limit

// PID state variables
volatile float pid_integral = 0.0f;
volatile float pid_previous_error = 0.0f;
volatile uint32_t pid_last_time = 0;

// Overshoot detection variables
volatile float pid_last_error_direction = 0.0f;
volatile uint8_t overshoot_detected = 0;
volatile uint8_t oscillation_detected = 0;  // ADD THIS LINE

// Rate limiting for smooth PID output
volatile float pid_output_filtered = 0.0f;
#define PID_RATE_LIMIT 2.0f  // Max change per update cycle (2%/cycle)

// SBUS recovery state tracking
volatile uint8_t sbus_was_lost = 0;         // Track if SBUS was previously lost
volatile uint8_t heading_lock_was_enabled = 0; // Remember if heading lock was enabled before SBUS loss

// Robot orientation offset - BNO085 sensor mounting compensation
#define ROBOT_FRONT_OFFSET 282.0f  // 102° + 180° = 282°

typedef struct {
	uint32_t magic_number;           // Validation marker
	float saved_locked_heading;      // Last locked heading value
	uint8_t heading_lock_was_active; // Was heading lock active when saved
} eeStorage_t;

eeStorage_t eeStorage = {0.0f, 0}; // Initialize storage with default values

#define EE_MAGIC_NUMBER 0x48454144  // "HEAD" in hex

// SH2 HAL glue
sh2_Hal_t sh2_hal;
sh2_SensorConfig_t sh2_config;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_IWDG_Init(void);
void StartDefaultTask(void *argument);
void StartCompassTask(void *argument);
void StartDisplayTask(void *argument);
void StartEncoderTask(void *argument);

/* USER CODE BEGIN PFP */
// Function prototypes
float compass_to_robot_front(float compass_heading);
float get_robot_front_heading(void);
void set_heading_lock(float compass_heading);
void disable_heading_lock(void);
float calculate_heading_pid_simplified(float target_heading, float current_heading);
void reset_heading_pid(void);
float angle_difference(float target, float current);
uint8_t is_sbus_signal_valid(void);
void save_heading_lock_to_eeprom(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Modified I2C bus scanner: stores found addresses in a string buffer
void i2c_bus_scan(char *found_devices, size_t bufsize) {
	char addr_str[8];
	uint8_t first = 1;
	found_devices[0] = '\0';

	for (uint8_t addr = 1; addr < 127; addr++) {
		if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK) {
			if (!first) {
				strncat(found_devices, ",",
						bufsize - strlen(found_devices) - 1);
			}
			snprintf(addr_str, sizeof(addr_str), "0x%02X", addr);
			strncat(found_devices, addr_str,
					bufsize - strlen(found_devices) - 1);
			first = 0;
		}
	}
}

// Read BNO085 Chip ID register (Register 0x00)
uint8_t bno085_read_chip_id() {
	uint8_t reg = 0x00;
	uint8_t id = 0;
	if (HAL_I2C_Master_Transmit(&hi2c1, BNO085_ADDR, &reg, 1, HAL_MAX_DELAY)
			!= HAL_OK)
		return 0xFF; // Indicate error
	if (HAL_I2C_Master_Receive(&hi2c1, BNO085_ADDR, &id, 1, HAL_MAX_DELAY)
			!= HAL_OK)
		return 0xFF; // Indicate error
	return id;
}

void sh2_event_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
	// Optional: handle async events (reset, etc.)
}

static void sh2_sensor_handler(void *cookie, sh2_SensorEvent_t *event) {
	// Handle sensor events from BNO085
	sh2_SensorValue_t value;
	if (sh2_decodeSensorEvent(&value, event) == 0) {
		if (value.sensorId == SH2_ROTATION_VECTOR) {
			float q0 = value.un.rotationVector.real;
			float q1 = value.un.rotationVector.i;
			float q2 = value.un.rotationVector.j;
			float q3 = value.un.rotationVector.k;
			float t3 = 2.0f * (q0 * q3 + q1 * q2);
			float t4 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
			float yaw = atan2f(t3, t4);
			float yaw_deg = yaw * (180.0f / (float) M_PI);
			if (yaw_deg < 0)
				yaw_deg += 360.0f;
			osMutexAcquire(CompassMutexHandle, osWaitForever);
			compass_heading_deg = yaw_deg;
			osMutexRelease(CompassMutexHandle);
		}
	}
}

void bno085_setup(void) {
	sh2_hal.open = sh2_hal_open;
	sh2_hal.close = sh2_hal_close;
	sh2_hal.read = sh2_hal_read;
	sh2_hal.write = sh2_hal_write;
	sh2_hal.getTimeUs = sh2_hal_getTimeUs;

	sh2_open(&sh2_hal, sh2_event_callback, NULL);
	sh2_setSensorCallback(sh2_sensor_handler, NULL);

	sh2_config.changeSensitivityEnabled = false;
	sh2_config.changeSensitivityRelative = false;
	sh2_config.wakeupEnabled = false;
	sh2_config.alwaysOnEnabled = false;
	sh2_config.sniffEnabled = false;
	sh2_config.changeSensitivity = 0;
	sh2_config.reportInterval_us = 10000; // 100Hz
	//sh2_config.reportInterval_us = 20000; // 50Hz
	sh2_config.batchInterval_us = 0;
	sh2_config.sensorSpecific = 0;

	sh2_setSensorConfig(SH2_ROTATION_VECTOR, &sh2_config);
}

// Convert SBUS channel value (172-1811) to percentage (-100 to +100)
int16_t sbus_to_percentage(uint16_t sbus_value) {
	if (sbus_value < 172)
		sbus_value = 172;
	if (sbus_value > 1811)
		sbus_value = 1811;

	int16_t result = (int16_t) (((int32_t) sbus_value - 992) * 100 / 820);

	// Optional: Dead zone of ±5%
	if (result >= -5 && result <= 5) {
		result = 0;
	}

	return result;
}

// Calculate individual motor speeds for 3-wheel omnidirectional robot
// x: Left/Right movement (-100 to +100) - Positive = Right, Negative = Left
// y: Forward/Backward movement (-100 to +100) - Positive = Forward, Negative = Backward
// rotation: Rotational movement (-100 to +100) - Positive = Clockwise, Negative = Counter-clockwise
void calculate_omni_motor_speeds(int16_t x, int16_t y, int16_t rotation,
		int16_t *left_front, int16_t *right_front, int16_t *back) {
	// 3-wheel omnidirectional kinematics - CORRECTED ROTATION DIRECTION
	// For 120-degree spaced wheels:
	// Left Front (Motor 1B):   +0.5*x + 0.866*y - rotation  (FIXED: was +rotation)
	// Right Front (Motor 1A):  +0.5*x - 0.866*y - rotation  (FIXED: was +rotation)
	// Back (Motor 2A):         -1.0*x + 0*y - rotation      (FIXED: was +rotation)

	// Use integer math (multiply by 100 to maintain precision, then divide)
	int32_t lf = (+50 * x + 87 * y - 100 * rotation) / 100; // Left Front (FIXED)
	int32_t rf = (+50 * x - 87 * y - 100 * rotation) / 100; // Right Front (FIXED)
	int32_t bk = (-100 * x + 0 * y - 100 * rotation) / 100;  // Back (FIXED)

	// Clamp to -100 to +100
	if (lf > 100)
		lf = 100;
	if (lf < -100)
		lf = -100;
	if (rf > 100)
		rf = 100;
	if (rf < -100)
		rf = -100;
	if (bk > 100)
		bk = 100;
	if (bk < -100)
		bk = -100;

	*left_front = (int16_t) lf;
	*right_front = (int16_t) rf;
	*back = (int16_t) bk;
}

// Calculate individual motor speeds for 3-wheel omnidirectional robot with heading lock
// x: Left/Right movement (-100 to +100) - Positive = Right, Negative = Left
// y: Forward/Backward movement (-100 to +100) - Positive = Forward, Negative = Backward
// rotation: Rotational movement (-100 to +100)
// heading_correction: PID output for heading lock (-100 to +100)
void calculate_omni_motor_speeds_with_heading(int16_t x, int16_t y,
		int16_t rotation, float heading_correction, int16_t *left_front,
		int16_t *right_front, int16_t *back) {

	// Blend manual rotation with heading correction
	// Give priority to manual input when operator is actively rotating
	float manual_rotation_f = (float) rotation;
	float total_rotation;

	if (fabs(manual_rotation_f) > 10.0f) {
		// Manual rotation is active - blend with less heading correction
		total_rotation = manual_rotation_f + (heading_correction * 0.3f);
	} else {
		// No manual rotation - use full heading correction
		total_rotation = manual_rotation_f + heading_correction;
	}

	// Clamp total rotation to prevent overflow
	if (total_rotation > 100.0f)
		total_rotation = 100.0f;
	if (total_rotation < -100.0f)
		total_rotation = -100.0f;

	// 3-wheel omnidirectional kinematics with heading correction - FIXED ROTATION
	int32_t lf = (+50 * x + 87 * y - (int32_t) total_rotation * 100) / 100; // FIXED
	int32_t rf = (+50 * x - 87 * y - (int32_t) total_rotation * 100) / 100; // FIXED
	int32_t bk = (-100 * x + 0 * y - (int32_t) total_rotation * 100) / 100; // FIXED

	// Clamp to -100 to +100
	if (lf > 100)
		lf = 100;
	if (lf < -100)
		lf = -100;
	if (rf > 100)
		rf = 100;
	if (rf < -100)
		rf = -100;
	if (bk > 100)
		bk = 100;
	if (bk < -100)
		bk = -100;

	*left_front = (int16_t) lf;
	*right_front = (int16_t) rf;
	*back = (int16_t) bk;
}

// Apply motor speeds to actual motors (3-wheel configuration)
void apply_motor_speeds_3wheel(int16_t left_front, int16_t right_front,
		int16_t back) {
	const uint16_t pwm_val1 = 49; // For TIM1 (100% duty if Period = 49)

	// Motor 1B - Left Front
	if (left_front == 0) {
		tb6612fng_brake(MOTOR_1B);
	} else if (left_front > 0) {
		tb6612fng_drive(MOTOR_1B, TB6612FNG_FORWARD,
				(pwm_val1 * left_front) / 100);
	} else {
		tb6612fng_drive(MOTOR_1B, TB6612FNG_BACKWARD,
				(pwm_val1 * (-left_front)) / 100);
	}

	// Motor 1A - Right Front
	if (right_front == 0) {
		tb6612fng_brake(MOTOR_1A);
	} else if (right_front > 0) {
		tb6612fng_drive(MOTOR_1A, TB6612FNG_FORWARD,
				(pwm_val1 * right_front) / 100);
	} else {
		tb6612fng_drive(MOTOR_1A, TB6612FNG_BACKWARD,
				(pwm_val1 * (-right_front)) / 100);
	}

	// Motor 2A - Back
	if (back == 0) {
		tb6612fng_brake(MOTOR_2A);
	} else if (back > 0) {
		tb6612fng_drive(MOTOR_2A, TB6612FNG_FORWARD, (pwm_val1 * back) / 100);
	} else {
		tb6612fng_drive(MOTOR_2A, TB6612FNG_BACKWARD,
				(pwm_val1 * (-back)) / 100);
	}
}

// Stop all motors (3-wheel configuration)
void stop_all_motors_3wheel(void) {
	tb6612fng_stop(MOTOR_1A); // Right Front
	tb6612fng_stop(MOTOR_1B); // Left Front
	tb6612fng_stop(MOTOR_2A); // Back
}

// Check if SBUS signal is valid and recent
// Enhanced SBUS signal check with recovery
uint8_t is_sbus_signal_valid(void) {
	uint32_t now = HAL_GetTick();

	// Check if we have received a valid frame recently
	if (sbus_signal_valid && (now - sbus_last_valid_time) <= SBUS_TIMEOUT_MS) {
		return 1;
	} else {
		sbus_signal_valid = 0; // Mark as invalid if timeout

		// If signal has been lost for a long time, try recovery
		if ((now - sbus_last_valid_time) > (SBUS_TIMEOUT_MS * 10)) {
			// Signal lost for 1 second - attempt recovery
			static uint32_t last_recovery_attempt = 0;
			if ((now - last_recovery_attempt) > 5000) {  // Try every 5 seconds
				last_recovery_attempt = now;

				// Reinitialize UART from main context (safer than interrupt)
				HAL_UART_AbortReceive(&huart2);
				MX_USART2_UART_Init();
				HAL_UART_Receive_DMA(&huart2, sbus_rx_buffer, SBUS_FRAME_SIZE);
			}
		}

		return 0;
	}
}

// Calculate shortest angle difference (handles 0-360 wrap-around)
float angle_difference(float target, float current) {
	float diff = target - current;

	// Normalize to -180 to +180 range
	while (diff > 180.0f)
		diff -= 360.0f;
	while (diff < -180.0f)
		diff += 360.0f;

	return diff;
}

// Simplified PID controller with reduced computational overhead
float calculate_heading_pid_simplified(float target_heading, float current_heading) {
    static float integral = 0.0f;
    static float previous_error = 0.0f;
    static float output_filtered = 0.0f;
    static uint32_t last_time = 0;
    static float error_history[3] = {0}; // Reduced from 5 to 3
    static uint8_t history_idx = 0;
    static uint32_t last_oscillation_time = 0;
    static uint32_t overshoot_cooldown = 0;

    uint32_t now = HAL_GetTick();
    if (last_time == 0) last_time = now;

    float dt = (now - last_time) / 1000.0f;
    if (dt <= 0) return output_filtered;

    // Calculate error with angle wrapping
    float error = target_heading - current_heading;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    // Store error in simplified history
    error_history[history_idx] = error;
    history_idx = (history_idx + 1) % 3;

    // Simplified oscillation detection (check only last 3 samples)
    uint8_t oscillation = 0;
    if (fabs(error) < 20.0f) { // Only check when close to target
        int sign_changes = 0;
        for (int i = 0; i < 2; i++) {
            if ((error_history[i] * error_history[i+1]) < 0) {
                sign_changes++;
            }
        }
        if (sign_changes >= 1 && (now - last_oscillation_time) > 1000) {
            oscillation = 1;
            last_oscillation_time = now;
            integral *= 0.5f; // Reduce integral windup
        }
    }

    // Simplified overshoot detection
    uint8_t overshoot = 0;
    if ((previous_error * error) < 0 && fabs(error) > 15.0f) {
        if ((now - overshoot_cooldown) > 1000) {
            overshoot = 1;
            overshoot_cooldown = now;
            integral *= 0.7f;
        }
    }

    // Simplified dead zone
    float dead_zone = oscillation ? 4.0f : (overshoot ? 2.0f : 1.0f);

    if (fabs(error) < dead_zone) {
        // Gentle decay near target
        integral *= 0.95f;
        output_filtered *= 0.9f;
        previous_error = error;
        last_time = now;
        return output_filtered;
    }

    // Simplified adaptive gains based on error magnitude only
    float kp, ki, kd, max_output;
    if (fabs(error) > 60.0f) {
        kp = 1.0f; ki = 0.02f; kd = 0.1f; max_output = 25.0f;
    } else if (fabs(error) > 20.0f) {
        kp = 1.5f; ki = 0.03f; kd = 0.15f; max_output = 20.0f;
    } else {
        kp = 2.0f; ki = 0.04f; kd = 0.2f; max_output = 15.0f;
    }

    // Reduce gains if problems detected
    if (overshoot) {
        kp *= 0.7f; ki *= 0.5f; kd *= 0.8f; max_output *= 0.8f;
    }
    if (oscillation) {
        kp *= 0.5f; ki *= 0.3f; kd *= 0.6f; max_output *= 0.7f;
    }

    // Calculate PID terms
    float proportional = kp * error;

    integral += error * dt;
    integral = fmaxf(-50.0f, fminf(50.0f, integral)); // Simple clamping
    float integral_term = ki * integral;

    float derivative = kd * (error - previous_error) / dt;

    float raw_output = proportional + integral_term + derivative;

    // Simple output limiting
    raw_output = fmaxf(-max_output, fminf(max_output, raw_output));

    // Simple rate limiting
    float rate_limit = fabs(error) > 30.0f ? 2.0f : 3.0f;
    if (overshoot) rate_limit *= 0.7f;
    if (oscillation) rate_limit *= 0.5f;

    float output_diff = raw_output - output_filtered;
    if (fabs(output_diff) > rate_limit) {
        output_diff = (output_diff > 0) ? rate_limit : -rate_limit;
    }
    output_filtered += output_diff;

    // Simple approach damping for small errors
    if (fabs(error) < 20.0f && fabs(error) > 3.0f) {
        float approach_factor = fabs(error) / 20.0f;
        if (approach_factor < 0.6f) approach_factor = 0.6f;
        output_filtered *= approach_factor;
    }

    // Update for next iteration
    previous_error = error;
    last_time = now;

    // Store for debugging
    heading_error = error;
    heading_pid_output = output_filtered;

    return output_filtered;
}

// Reset PID controller
void reset_heading_pid(void) {
	pid_integral = 0.0f;
	pid_previous_error = 0.0f;
	pid_output_filtered = 0.0f;  // Reset filtered output
	pid_last_time = HAL_GetTick();
	heading_error = 0.0f;
	heading_pid_output = 0.0f;
}
// Updated set heading lock function
void set_heading_lock(float compass_heading) {
	locked_heading = compass_to_robot_front(compass_heading);
	heading_lock_enabled = 1;
	reset_heading_pid();
	save_heading_lock_to_eeprom(); // Add this line
}

// Disable heading lock
void disable_heading_lock(void) {
	heading_lock_enabled = 0;
	reset_heading_pid();
}

// Convert compass heading to robot front direction
float compass_to_robot_front(float compass_heading) {
	float robot_front = compass_heading + ROBOT_FRONT_OFFSET;

	// Normalize to 0-360 range
	while (robot_front >= 360.0f) {
		robot_front -= 360.0f;
	}
	while (robot_front < 0.0f) {
		robot_front += 360.0f;
	}

	return robot_front;
}

// Get current robot front direction
float get_robot_front_heading(void) {
	float compass_heading;

	osMutexAcquire(CompassMutexHandle, osWaitForever);
	compass_heading = compass_heading_deg;
	osMutexRelease(CompassMutexHandle);

	return compass_to_robot_front(compass_heading);
}

// EEPROM storage functions for heading lock
void save_heading_lock_to_eeprom(void) {
	if (heading_lock_enabled) {
		osMutexAcquire(EEPROMMutexHandle, osWaitForever);
		eeStorage.magic_number = EE_MAGIC_NUMBER;
		eeStorage.saved_locked_heading = locked_heading;
		eeStorage.heading_lock_was_active = 1;
		EE_Write();
		osMutexRelease(EEPROMMutexHandle);
	}
}

bool restore_heading_lock_from_eeprom(void) {
	bool result = false;
	osMutexAcquire(EEPROMMutexHandle, osWaitForever);
	EE_Read();
	if (eeStorage.magic_number == EE_MAGIC_NUMBER &&
			eeStorage.heading_lock_was_active) {
		locked_heading = eeStorage.saved_locked_heading;
		result = true;
	}
	osMutexRelease(EEPROMMutexHandle);
	return result;
}

// Get the current position of channel 7 (assumed to be used for mode selection)
ch7_position_t get_channel7_position(void) {
	if (!is_sbus_signal_valid()) {
		return CH7_POSITION_UNKNOWN;
	}

	uint16_t ch7_value = sbus_channels[6]; // Channel 7 (index 6)

	// Define positions with dead zones (assuming SBUS range 172-1811)
	// Min: 172-500, Mid: 700-1283, Max: 1511-1811
	if (ch7_value >= 172 && ch7_value <= 500) {
		return CH7_POSITION_MIN;
	} else if (ch7_value >= 700 && ch7_value <= 1283) {
		return CH7_POSITION_MID;
	} else if (ch7_value >= 1511 && ch7_value <= 1811) {
		return CH7_POSITION_MAX;
	}

	return CH7_POSITION_UNKNOWN; // In dead zone or invalid
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_DMA(&huart2, sbus_rx_buffer, SBUS_FRAME_SIZE);

	TB6612FNG_Init(); // Initialize TB6612 drivers and PWM

	EE_Init(&eeStorage, sizeof(eeStorage_t)); // Initialize EEPROM emulation with storage structure

	char i2c_devices[128];
	//  char display_str[48];

	// Set PC13 high to turn the onboard LED on
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	HAL_Delay(BNO085_BOOT_DELAY_MS);

	// Set PC13 high to turn the onboard LED off
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

	i2c_bus_scan(i2c_devices, sizeof(i2c_devices)); // Scan and store result

	uint8_t chip_id = bno085_read_chip_id();
	if (chip_id == 0xFF) {
		// Optionally display error on OLED
		Error_Handler();
	}

	bno085_setup();
	//HAL_Delay(100); // Give BNO085 time to boot
	sh2_clearTare(); // Now device uses absolute/magnetic north as reference

	// Init lcd using one of the stm32HAL i2c typedefs
	if (ssd1306_Init(&hi2c1) != 0) {
		Error_Handler();
	}

	ssd1306_Fill(Black);
	ssd1306_UpdateScreen(&hi2c1);

    HAL_IWDG_Refresh(&hiwdg); // Refresh watchdog
	HAL_Delay(1000);

	// Write data to local screenbuffer
	ssd1306_SetCursor(1, 0);
	ssd1306_WriteString(APP_VERSION, Font_7x10, White);
	ssd1306_SetCursor(1, 12);
	if (i2c_devices[0] == '\0') {
		ssd1306_WriteString("No I2C found", Font_7x10, White);
	} else {
		ssd1306_WriteString("I2C:", Font_7x10, White);
		ssd1306_SetCursor(1, 24);
		ssd1306_WriteString(i2c_devices, Font_7x10, White);
	}

	// Copy all data from local screenbuffer to the screen
	ssd1306_UpdateScreen(&hi2c1);

    HAL_IWDG_Refresh(&hiwdg); // Refresh watchdog
	// Delay to see I2C info
	HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of CompassMutex */
  CompassMutexHandle = osMutexNew(&CompassMutex_attributes);

  /* creation of I2C1Mutex */
  I2C1MutexHandle = osMutexNew(&I2C1Mutex_attributes);

  /* creation of EncoderMutex */
  EncoderMutexHandle = osMutexNew(&EncoderMutex_attributes);

  /* creation of UARTMutex */
  UARTMutexHandle = osMutexNew(&UARTMutex_attributes);

  /* creation of ModeMutex */
  ModeMutexHandle = osMutexNew(&ModeMutex_attributes);

  /* creation of EEPROMMutex */
  EEPROMMutexHandle = osMutexNew(&EEPROMMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of CompassTask */
  CompassTaskHandle = osThreadNew(StartCompassTask, NULL, &CompassTask_attributes);

  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(StartDisplayTask, NULL, &DisplayTask_attributes);

  /* creation of EncoderTask */
  EncoderTaskHandle = osThreadNew(StartEncoderTask, NULL, &EncoderTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		osDelay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 3125;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 49;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 24;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 100000;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_2;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Motor_1A_IN1_Pin|Motor_1A_IN2_Pin|Motor_1B_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Motor_2A_IN1_Pin|Motor_2A_IN2_Pin|TB6612__1_STBY_Pin|TB6612__2_STBY_Pin
                          |Motor_2B_IN1_Pin|Motor_2B_IN2_Pin|Motor_1B_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor_1A_IN1_Pin Motor_1A_IN2_Pin Motor_1B_IN2_Pin */
  GPIO_InitStruct.Pin = Motor_1A_IN1_Pin|Motor_1A_IN2_Pin|Motor_1B_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_for_CLK___SNP00128_Pin */
  GPIO_InitStruct.Pin = INT_for_CLK___SNP00128_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(INT_for_CLK___SNP00128_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : for_DT___SNP00128_Pin */
  GPIO_InitStruct.Pin = for_DT___SNP00128_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(for_DT___SNP00128_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_for_SW___SNP00128_Pin */
  GPIO_InitStruct.Pin = INT_for_SW___SNP00128_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(INT_for_SW___SNP00128_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor_2A_IN1_Pin Motor_2A_IN2_Pin TB6612__1_STBY_Pin TB6612__2_STBY_Pin
                           Motor_2B_IN1_Pin Motor_2B_IN2_Pin Motor_1B_IN1_Pin */
  GPIO_InitStruct.Pin = Motor_2A_IN1_Pin|Motor_2A_IN2_Pin|TB6612__1_STBY_Pin|TB6612__2_STBY_Pin
                          |Motor_2B_IN1_Pin|Motor_2B_IN2_Pin|Motor_1B_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_for_BNO085_Pin */
  GPIO_InitStruct.Pin = INT_for_BNO085_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_for_BNO085_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// EXTI Callback for INT pin (e.g., PA0 for EXTI0)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	uint32_t now = HAL_GetTick();

	// BNO085 INT (PA8)
	if (GPIO_Pin == GPIO_PIN_8) {
		bno085_int_flag = 1;
	}
	if (GPIO_Pin == ENC_CLK_Pin) {
		if (HAL_GPIO_ReadPin(ENC_CLK_GPIO_Port, ENC_CLK_Pin)
				== GPIO_PIN_RESET) {
			if (now - last_clk_tick > ENCODER_DEBOUNCE_MS) {
				last_clk_tick = now;
				osMutexAcquire(EncoderMutexHandle, 0);
				if (HAL_GPIO_ReadPin(ENC_DT_GPIO_Port, ENC_DT_Pin)
						== GPIO_PIN_SET)
					encoder_count++;
				else
					encoder_count--;
				osMutexRelease(EncoderMutexHandle);
			}
		}
	}

	if (GPIO_Pin == ENC_SW_Pin) {
		GPIO_PinState clk_state = HAL_GPIO_ReadPin(ENC_CLK_GPIO_Port,
				ENC_CLK_Pin);
		GPIO_PinState dt_state = HAL_GPIO_ReadPin(ENC_DT_GPIO_Port,
				ENC_DT_Pin);
		GPIO_PinState sw_state = HAL_GPIO_ReadPin(ENC_SW_GPIO_Port,
				ENC_SW_Pin);
		// Only count if button is actually pressed (logic low) and both CLK and DT are high
		// Note: When turn rotary knob left/right, SW pin is always pulled low together with CLK and DT pins.
		if (sw_state == GPIO_PIN_RESET) {
			if (dt_state == GPIO_PIN_SET) {
				if (clk_state == GPIO_PIN_SET) {
					// Ignore SW if just rotated (within 10 ms of last CLK)
					if ((now - last_clk_tick) > CLK_SW_DEBOUNCE_MS) {
						if (now - last_sw_tick > BUTTON_DEBOUNCE_MS) {
							last_sw_tick = now;
							osMutexAcquire(EncoderMutexHandle, 0);
							encoder_button_pressed = 1;
							encoder_sw_count++; // Increment counter on each debounced press
							osMutexRelease(EncoderMutexHandle);
						}
					}
				}
			}
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		// Validate SBUS frame start and end bytes
		if (sbus_rx_buffer[0] == 0x0F && sbus_rx_buffer[24] == 0x00) {
			// Valid SBUS frame received
			sbus_last_valid_time = HAL_GetTick();
			sbus_signal_valid = 1;

			// Unpack all 16 channels
			for (int ch = 0; ch < 16; ch++) {
				int byte_idx = 1 + (ch * 11) / 8;
				int bit_idx = (ch * 11) % 8;
				uint16_t value = (sbus_rx_buffer[byte_idx]
												 | (sbus_rx_buffer[byte_idx + 1] << 8)
												 | (sbus_rx_buffer[byte_idx + 2] << 16));
				value = (value >> bit_idx) & 0x07FF;

				// Additional range validation
				if (value >= 172 && value <= 1811) {
					sbus_channels[ch] = value;
				}
			}
		} else {
			// Invalid frame - don't update sbus_signal_valid or channels
		}
		// DO NOT restart DMA - Circular mode handles this automatically
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // Indicate error visually
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

        // Mark SBUS as invalid on UART error
        sbus_signal_valid = 0;

        // Stop all motors immediately for safety
        stop_all_motors_3wheel();

        // Store error information
        osMutexAcquire(UARTMutexHandle, osWaitForever);
        uart2_error_code = huart->ErrorCode;
        uart2_error_flag = 1;
        osMutexRelease(UARTMutexHandle);

        // Set flag for recovery in main task context
        sbus_recovery_requested = 1;
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
    static ch7_position_t last_ch7_position = CH7_POSITION_UNKNOWN;
    static uint32_t ch7_stable_time = 0;
    const uint32_t CH7_DEBOUNCE_MS = 100; // Debounce time for channel 7

    // --- SBUS recovery flag ---
    extern volatile uint8_t sbus_recovery_requested; // Declare if not already global

    /* Infinite loop */
    for (;;) {
        robot_mode_t mode;
        robot_state_t state;
        uint8_t mode_changed = 0;
        uint8_t state_changed = 0;
        uint8_t sbus_valid = 0;
        float current_compass_heading = 0.0f;
        float current_robot_front = 0.0f;

        // --- SBUS Recovery Logic ---
        if (sbus_recovery_requested) {
            sbus_recovery_requested = 0;
            // Perform recovery attempts (non-blocking, watchdog-friendly)
            for (int attempt = 0; attempt < SBUS_MAX_RECOVERY_ATTEMPTS; attempt++) {
                HAL_UART_AbortReceive(&huart2);
                MX_USART2_UART_Init();
                HAL_UART_Receive_DMA(&huart2, sbus_rx_buffer, SBUS_FRAME_SIZE);
                HAL_IWDG_Refresh(&hiwdg); // Refresh watchdog during recovery
                osDelay(10); // Give hardware time to settle
                // Optionally check if recovery succeeded and break if so
            }
        }

        // Check SBUS signal validity
        sbus_valid = is_sbus_signal_valid();

        // Get current compass heading
        osMutexAcquire(CompassMutexHandle, osWaitForever);
        current_compass_heading = compass_heading_deg;
        osMutexRelease(CompassMutexHandle);

        // Convert to robot front direction
        current_robot_front = compass_to_robot_front(current_compass_heading);

        // Check for mode/state changes
        osMutexAcquire(ModeMutexHandle, osWaitForever);
        mode = current_mode;
        state = robot_state;
        if (mode_change_request) {
            mode_change_request = 0;
            mode_changed = 1;
        }
        if (state_toggle_request) {
            state_toggle_request = 0;
            state_changed = 1;
        }
        osMutexRelease(ModeMutexHandle);

        // ENHANCED: Check Channel 7 for automatic mode control
        if (sbus_valid) {
            ch7_position_t current_ch7 = get_channel7_position();
            uint32_t now = HAL_GetTick();

            // Debounce Channel 7 changes
            if (current_ch7 != last_ch7_position && current_ch7 != CH7_POSITION_UNKNOWN) {
                if (ch7_stable_time == 0) {
                    ch7_stable_time = now; // Start debounce timer
                } else if ((now - ch7_stable_time) >= CH7_DEBOUNCE_MS) {
                    // Channel 7 position has been stable for debounce time
                    robot_mode_t target_mode;
                    robot_state_t target_state;

                    switch (current_ch7) {
                    case CH7_POSITION_MIN:
                        target_mode = MODE_DISPLAY_SENSORS;
                        target_state = STATE_IDLE;
                        break;
                    case CH7_POSITION_MID:
                        target_mode = MODE_MOVEMENT;
                        target_state = STATE_RUNNING;
                        break;
                    case CH7_POSITION_MAX:
                        target_mode = MODE_MOVEMENT_HL;
                        target_state = STATE_RUNNING;
                        break;
                    default:
                        target_mode = mode; // No change
                        target_state = state;
                        break;
                    }

                    // Apply changes if different from current
                    osMutexAcquire(ModeMutexHandle, osWaitForever);
                    if (target_mode != current_mode || target_state != robot_state) {
                        current_mode = target_mode;
                        selected_mode = target_mode; // Keep encoder selection in sync
                        robot_state = target_state;
                        mode_change_request = 1;
                        state_toggle_request = 1;
                        mode_changed = 1;
                        state_changed = 1;
                    }
                    osMutexRelease(ModeMutexHandle);

                    last_ch7_position = current_ch7;
                    ch7_stable_time = 0; // Reset debounce timer
                }
            } else if (current_ch7 == last_ch7_position) {
                ch7_stable_time = 0; // Reset debounce if position unchanged
            }
        } else {
            // SBUS invalid - reset Channel 7 tracking
            last_ch7_position = CH7_POSITION_UNKNOWN;
            ch7_stable_time = 0;
        }

        // Handle mode changes
        if (mode_changed) {
            // Stop all motors when changing modes
            stop_all_motors_3wheel();
            // Disable heading lock when changing modes
            disable_heading_lock();
            // Reset recovery state
            sbus_was_lost = 0;
            heading_lock_was_enabled = 0;
        }

        // Handle state changes
        if (state_changed) {
            if (state == STATE_IDLE) {
                // Stop all motors when going to idle
                stop_all_motors_3wheel();
                // Disable heading lock when going to idle
                disable_heading_lock();
                // Reset recovery state
                sbus_was_lost = 0;
                heading_lock_was_enabled = 0;
            } else if (state == STATE_RUNNING && mode == MODE_MOVEMENT_HL) { // UPDATED
                // Enable heading lock when starting movement mode with heading lock
                // Use current compass heading (function will convert to robot front)
                set_heading_lock(current_compass_heading);
                // Reset recovery state for new session
                sbus_was_lost = 0;
                heading_lock_was_enabled = 1;
            }
        }

        // ENHANCED SBUS RECOVERY LOGIC
        if (mode == MODE_MOVEMENT || mode == MODE_MOVEMENT_HL) { // UPDATED
            if (sbus_valid) {
                // SBUS signal is valid

                // Check if we just recovered from SBUS loss
                if (sbus_was_lost && mode == MODE_MOVEMENT_HL && state == STATE_RUNNING) {
                    if (heading_lock_was_enabled) {
                        // Try to restore from EEPROM first
                        if (restore_heading_lock_from_eeprom()) {
                            heading_lock_enabled = 1;
                            reset_heading_pid();
                        } else {
                            // Fallback to current heading
                            set_heading_lock(current_compass_heading);
                        }
                    }
                    sbus_was_lost = 0;
                }

                // Updated SBUS channel mapping:
                // Channel 2 (index 1): Left/Right (X-axis)
                // Channel 5 (index 4): Forward/Backward (Y-axis)
                // Channel 4 (index 3): Rotation (Z-axis)
                int16_t x = sbus_to_percentage(sbus_channels[1]); // Channel 2: Left/Right
                int16_t y = sbus_to_percentage(sbus_channels[4]); // Channel 5: Forward/Backward
                int16_t rot = sbus_to_percentage(sbus_channels[3]); // Channel 4: Rotation

                // Calculate heading correction if heading lock is enabled
                float heading_correction = 0.0f;
                if (heading_lock_enabled && mode == MODE_MOVEMENT_HL) { // UPDATED
                    heading_correction = calculate_heading_pid_simplified(locked_heading, current_robot_front);
                }

                // Calculate individual motor speeds for 3-wheel omnidirectional
                int16_t lf, rf, bk;
                if (heading_lock_enabled && mode == MODE_MOVEMENT_HL) { // UPDATED
                    calculate_omni_motor_speeds_with_heading(x, y, rot,
                            heading_correction, &lf, &rf, &bk);
                } else {
                    calculate_omni_motor_speeds(x, y, rot, &lf, &rf, &bk);
                }

                // Update shared variables
                joystick_x = x;
                joystick_y = y;
                joystick_rotation = rot;
                motor_left_front = lf;   // Motor 1B
                motor_right_front = rf;  // Motor 1A
                motor_back = bk;         // Motor 2A

                // Apply to actual motors for both movement modes when running
                if ((mode == MODE_MOVEMENT || mode == MODE_MOVEMENT_HL)
                        && state == STATE_RUNNING) { // UPDATED
                    apply_motor_speeds_3wheel(lf, rf, bk);
                }
            } else {
                // SBUS signal lost

                // Remember heading lock state before disabling it
                if (!sbus_was_lost) {
                    // First time detecting SBUS loss
                    heading_lock_was_enabled = heading_lock_enabled;
                    sbus_was_lost = 1;
                }

                // Stop all motors immediately for safety
                stop_all_motors_3wheel();
                // Temporarily disable heading lock (but remember its state)
                disable_heading_lock();

                // Reset joystick values to indicate no signal
                joystick_x = 0;
                joystick_y = 0;
                joystick_rotation = 0;
                motor_left_front = 0;
                motor_right_front = 0;
                motor_back = 0;
            }
        }

        HAL_IWDG_Refresh(&hiwdg); // Refresh watchdog

        osDelay(50); // Update at 20Hz
    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCompassTask */
/**
 * @brief Function implementing the CompassTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCompassTask */
void StartCompassTask(void *argument)
{
  /* USER CODE BEGIN StartCompassTask */
	/* Infinite loop */
	for (;;) {
		if (bno085_int_flag) {
			bno085_int_flag = 0;
			sh2_service();
		}
		osDelay(10);
	}
  /* USER CODE END StartCompassTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
 * @brief Function implementing the DisplayTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN StartDisplayTask */
	char display_str[48];
	const char *mode_names[] = { "SENSOR", "MOVE", "MOVE_HL" }; // UPDATED NAMES
	const char *state_names[] = { "IDLE", "RUN" };
	/* Infinite loop */
	for (;;) {
		robot_mode_t mode;
		robot_mode_t sel_mode;
		robot_state_t state;
		float heading;
		int32_t sw_count, enc_count;
		uint8_t show_uart_error = 0;
		uint8_t sbus_valid = 0;
		uint32_t error_code = 0;

		// Check SBUS signal validity
		sbus_valid = is_sbus_signal_valid();

		// Check for UART error under mutex
		osMutexAcquire(UARTMutexHandle, osWaitForever);
		if (uart2_error_flag) {
			show_uart_error = 1;
			error_code = uart2_error_code;
			uart2_error_flag = 0; // Clear after displaying
		}
		osMutexRelease(UARTMutexHandle);

		if (show_uart_error) {
			char err_str[32];
			// Show more detailed error information
			snprintf(err_str, sizeof(err_str), "UART Err: 0x%lX A:%d",
					error_code, sbus_recovery_attempts);
			ssd1306_Fill(Black);
			ssd1306_SetCursor(1, 0);
			ssd1306_WriteString(err_str, Font_7x10, White);

			// Show recovery status
			snprintf(err_str, sizeof(err_str), "Recovery: %s",
					(sbus_recovery_attempts < SBUS_MAX_RECOVERY_ATTEMPTS) ?
							"TRYING" : "FAILED");
			ssd1306_SetCursor(1, 12);
			ssd1306_WriteString(err_str, Font_7x10, White);

			ssd1306_UpdateScreen(&hi2c1);
			osDelay(2000); // Show error for 2s
			continue;
		}

		// Get current mode, selected mode, and state
		osMutexAcquire(ModeMutexHandle, osWaitForever);
		mode = current_mode;
		sel_mode = selected_mode;
		state = robot_state;
		osMutexRelease(ModeMutexHandle);

		// Get compass heading
		osMutexAcquire(CompassMutexHandle, osWaitForever);
		heading = compass_heading_deg;
		osMutexRelease(CompassMutexHandle);

		// Get encoder data
		osMutexAcquire(EncoderMutexHandle, osWaitForever);
		sw_count = encoder_sw_count;
		enc_count = encoder_count;
		osMutexRelease(EncoderMutexHandle);

		ssd1306_Fill(Black);

		// Line 1: Show selected mode with indicator if different from current
		// Add SBUS status, recovery status, and heading lock indicator
		if (sel_mode == mode) {
			// Current mode - show with state, SBUS status, recovery status, and heading lock
			char sbus_status = sbus_valid ? 'S' : '!';
			char recovery_status = sbus_was_lost ? 'R' : ' '; // R = recovering
			char heading_status =
					(heading_lock_enabled && mode == MODE_MOVEMENT_HL) ?
							'H' : ' '; // UPDATED

			snprintf(display_str, sizeof(display_str), "%s %s %c%c%c",
					mode_names[mode], state_names[state], sbus_status,
					recovery_status, heading_status);
		} else {
			// Selected mode (different from current) - show with arrow indicator
			char sbus_status = sbus_valid ? 'S' : '!';
			char recovery_status = sbus_was_lost ? 'R' : ' ';
			char heading_status =
					(heading_lock_enabled && mode == MODE_MOVEMENT_HL) ?
							'H' : ' '; // UPDATED

			snprintf(display_str, sizeof(display_str), ">%s< %s %c%c%c",
					mode_names[sel_mode], state_names[state], sbus_status,
					recovery_status, heading_status);
		}
		ssd1306_SetCursor(1, 0);
		ssd1306_WriteString(display_str, Font_7x10, White);

		// Display content based on current active mode (not selected mode)
		switch (mode) {
		case MODE_DISPLAY_SENSORS:
		    // Line 2: Show both compass and robot front headings
		    float robot_front = compass_to_robot_front(heading);
		    snprintf(display_str, sizeof(display_str), "C:%.0f R:%.0f deg",
		            heading, robot_front);
		    ssd1306_SetCursor(1, 12);
		    ssd1306_WriteString(display_str, Font_7x10, White);

		    // Line 3: Encoder data and Channel 7 position
		    const char* ch7_names[] = {"MIN", "MID", "MAX", "???"};
		    ch7_position_t ch7_pos = sbus_valid ? get_channel7_position() : CH7_POSITION_UNKNOWN;
		    snprintf(display_str, sizeof(display_str), "SW:%ld ENC:%ld CH7:%s",
		            (long)sw_count, (long)enc_count, ch7_names[ch7_pos]);
		    ssd1306_SetCursor(1, 24);
		    ssd1306_WriteString(display_str, Font_7x10, White);

		    // Line 4: SBUS channels including Channel 7 raw value
		    if (sbus_valid) {
		        int16_t ch2_pct = sbus_to_percentage(sbus_channels[1]);
		        int16_t ch5_pct = sbus_to_percentage(sbus_channels[4]);
		        int16_t ch4_pct = sbus_to_percentage(sbus_channels[3]);
		        snprintf(display_str, sizeof(display_str), "2:%3d 5:%3d 4:%3d",
		                ch2_pct, ch5_pct, ch4_pct);
		    } else {
		        snprintf(display_str, sizeof(display_str), "SBUS SIGNAL LOST");
		    }
		    ssd1306_SetCursor(1, 36);
		    ssd1306_WriteString(display_str, Font_7x10, White);
		    break;

		case MODE_MOVEMENT:        // UPDATED - was MODE_MOVEMENT_SIMULATOR
		case MODE_MOVEMENT_HL:     // UPDATED - was MODE_MOVEMENT
			if (sbus_valid) {
				// Line 2: Joystick input values (LR=Left/Right, FB=Forward/Backward, R=Rotation)
				snprintf(display_str, sizeof(display_str),
						"LR:%3d FB:%3d R:%3d", joystick_x, joystick_y,
						joystick_rotation);
				ssd1306_SetCursor(1, 12);
				ssd1306_WriteString(display_str, Font_7x10, White);

				// Line 3: Show heading lock info with oscillation detection
				if (heading_lock_enabled && mode == MODE_MOVEMENT_HL) { // UPDATED
					float current_robot_front = compass_to_robot_front(heading);

					// Enhanced status indicators
					char status_char = ' ';
					if (oscillation_detected) {
						status_char = '~'; // Oscillation detected
					} else if (overshoot_detected) {
						status_char = 'O'; // Overshoot detected
					} else if (fabs(heading_error) > 2.0f) { // Reduced threshold
						status_char = (heading_pid_output > 0) ? 'R' : 'L'; // Turning Right/Left
					} else {
						status_char = '='; // Near target
					}

					snprintf(display_str, sizeof(display_str),
							"L:%.0f N:%.0f E:%.1f%c", locked_heading,
							current_robot_front, heading_error, status_char);
				} else {
					// Show motor values
					snprintf(display_str, sizeof(display_str), "LF:%4d RF:%4d",
							motor_left_front, motor_right_front);
				}
				ssd1306_SetCursor(1, 24);
				ssd1306_WriteString(display_str, Font_7x10, White);

				// Line 4: PID output with overshoot indicator
				if (heading_lock_enabled && mode == MODE_MOVEMENT_HL) { // UPDATED
					snprintf(display_str, sizeof(display_str),
							"PID:%.1f BK:%4d%s", heading_pid_output, motor_back,
							overshoot_detected ? " OS" : "");
				} else {
					snprintf(display_str, sizeof(display_str), "BK:%4d",
							motor_back);
				}
				ssd1306_SetCursor(1, 36);
				ssd1306_WriteString(display_str, Font_7x10, White);
			} else {
				// SBUS signal lost - show warning
				snprintf(display_str, sizeof(display_str), "SBUS SIGNAL LOST");
				ssd1306_SetCursor(1, 12);
				ssd1306_WriteString(display_str, Font_7x10, White);

				snprintf(display_str, sizeof(display_str), "MOTORS STOPPED");
				ssd1306_SetCursor(1, 24);
				ssd1306_WriteString(display_str, Font_7x10, White);

				snprintf(display_str, sizeof(display_str), "FOR SAFETY");
				ssd1306_SetCursor(1, 36);
				ssd1306_WriteString(display_str, Font_7x10, White);
			}
			break;

		case MODE_COUNT:
		default:
			// Handle invalid mode - should never happen
			snprintf(display_str, sizeof(display_str), "INVALID MODE");
			ssd1306_SetCursor(1, 12);
			ssd1306_WriteString(display_str, Font_7x10, White);
			break;
		}

		ssd1306_UpdateScreen(&hi2c1);
		//osDelay(100);
		osDelay(200); // Update display every 200ms (5Hz) for smoother updates
	}
  /* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_StartEncoderTask */
/**
 * @brief Function implementing the EncoderTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartEncoderTask */
void StartEncoderTask(void *argument)
{
  /* USER CODE BEGIN StartEncoderTask */
	int32_t last_count = 0;
	robot_mode_t temp_mode = MODE_DISPLAY_SENSORS;
	/* Infinite loop */
	for (;;) {
		osMutexAcquire(EncoderMutexHandle, osWaitForever);
		int32_t count = encoder_count;
		uint8_t button = encoder_button_pressed;
		if (button)
			encoder_button_pressed = 0;
		osMutexRelease(EncoderMutexHandle);

		if (count != last_count) {
			// Handle encoder movement - change mode selection
			int32_t diff = count - last_count;
			last_count = count;

			osMutexAcquire(ModeMutexHandle, osWaitForever);
			robot_state_t state = robot_state;
			osMutexRelease(ModeMutexHandle);

			// Only allow mode change when robot is idle
			if (state == STATE_IDLE) {
				if (diff > 0) {
					temp_mode = (robot_mode_t) ((temp_mode + 1) % MODE_COUNT);
				} else {
					temp_mode = (robot_mode_t) ((temp_mode + MODE_COUNT - 1)
							% MODE_COUNT);
				}

				// Update the shared selected_mode variable
				osMutexAcquire(ModeMutexHandle, osWaitForever);
				selected_mode = temp_mode;
				osMutexRelease(ModeMutexHandle);
			}
		}

		if (button) {
			// Handle button press
			osMutexAcquire(ModeMutexHandle, osWaitForever);
			robot_mode_t current = current_mode;
			robot_state_t state = robot_state;

			if (state == STATE_IDLE) {
				// If idle, either change mode or start movement (for movement modes)
				if (temp_mode != current) {
					// Change mode
					current_mode = temp_mode;
					selected_mode = temp_mode;  // Keep them in sync
					mode_change_request = 1;
				} else if (current == MODE_MOVEMENT // UPDATED
						|| current == MODE_MOVEMENT_HL) { // UPDATED
					// Start movement
					robot_state = STATE_RUNNING;
					state_toggle_request = 1;
				}
			} else {
				// If running, stop movement
				robot_state = STATE_IDLE;
				state_toggle_request = 1;
			}
			osMutexRelease(ModeMutexHandle);
		}
		//osDelay(10);
		osDelay(20); // Update encoder every 20ms (50Hz)
	}
  /* USER CODE END StartEncoderTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	// Don't disable interrupts - allow system to continue
	// __disable_irq();  // ❌ REMOVE THIS LINE
	// Enable GPIOC clock if not already enabled
	__HAL_RCC_GPIOC_CLK_ENABLE();

	// Flash LED to indicate error, but don't get stuck
	for (int i = 0; i < 10; i++) {  // Flash 10 times then continue
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(100);  // Shorter delay
	}

	// Instead of infinite loop, try to recover
	// Reset system if this is a critical error
	NVIC_SystemReset();
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
