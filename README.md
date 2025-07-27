# STM32F411CEU6 Omniwheels Robot (3-Wheel Configuration)

This repository contains the firmware and source code for an advanced omniwheels robot with a 3-wheel configuration, powered by an STM32F411CEU6 microcontroller. The project uses STM32CubeIDE and FreeRTOS, featuring real-time control, heading lock functionality, and comprehensive sensor integration.

---

## Features

### Core Functionality
- **3-Wheel Omnidirectional Movement** with precision motor control
- **Heading Lock System** with advanced PID control and oscillation detection
- **SBUS RC Control** with automatic signal recovery and failsafe
- **Multiple Operating Modes** with automatic mode switching via RC channel or encoder
- **Real-time Sensor Fusion** using BNO085 IMU for accurate orientation

### Hardware Integration
- **SSD1306 OLED Display** for real-time status and debugging
- **BNO085 IMU Sensor** for 9-DOF motion sensing and compass heading
- **Rotary Encoder Interface** for manual mode selection and control
- **Dual TB6612FNG Motor Drivers** for precise motor control
- **FrSky R-XSR Receiver with SBUS Inverter** for reliable SBUS RC signal input
- **Watchdog Timer** for system reliability and fault recovery
- **FRAM (MB85RC256V)** for persistent settings storage and recovery via I2C1

### Software Features
- **FreeRTOS** real-time operating system with mutex protection
- **Advanced PID Controller** with adaptive gains and anti-windup
- **Signal Recovery Systems** for SBUS and sensor fault tolerance
- **Modular Architecture** for easy maintenance and feature expansion

---

## Hardware Requirements

### Core Components
- STM32F411CEU6 "Black Pill" development board
- 3 × Omniwheels with DC motors
- 2 × TB6612FNG dual motor driver modules
- SSD1306 128×64 OLED display (I2C)
- BNO085 9-DOF IMU sensor (I2C)
- MB85RC256V FRAM module (I2C1) for persistent storage
- Rotary encoder with push button
- SBUS receiver (FrSky or compatible)
- SBUS signal inverter circuit

### Power Requirements
- **Source:** 2-cell LiPo battery (nominal 7.4V, fully charged 8.4V)
- **Regulation:** Buck converter to 5V for STM32 board and peripherals
- **Motors:** 2S direct or via separate regulator (ensure current rating matches motor load)
- **Logic:** 5V regulated supply for STM32, sensors, and display

### Development Tools
- ST-Link V2 or compatible programmer/debugger
- STM32CubeIDE (latest version recommended)
- RC transmitter with SBUS output capability

---

## Library Overview

### Main Libraries Used

- **STM32 HAL (Hardware Abstraction Layer):**  
  For all low-level peripheral access (I2C, UART, GPIO, TIM, DMA, etc.).
- **FreeRTOS:**  
  Real-time operating system for multitasking, mutexes, and thread management.
- **SSD1306 Library:**  
  For driving the 128×64 OLED display over I2C.
- **BNO085 SH2 Library:**  
  For communication and sensor fusion data from the BNO085 IMU (via I2C, using SH2 protocol).
- **TB6612FNG Motor Driver Library:**  
  For controlling dual H-bridge motor drivers.
- **FRAM (MB85RC256V) Library:**  
  For persistent storage of heading lock and other key data using the MB85RC256V FRAM chip via I2C1.
- **Custom SBUS Decoder:**  
  For parsing SBUS frames from the RC receiver using DMA and error recovery.

---

## MCU Clock Configuration (Optimized for 100MHz)

The system is configured to operate at a **100MHz core frequency**, optimizing all subsystems and peripherals for this speed.

### Clock Source and PLL Settings
- **Clock Source:** External crystal oscillator (HSE)
- **HSE Frequency:** 25MHz (STM32F411CEU6 Black Pill standard crystal)
- **PLL Configuration:**
  - **PLL Source:** HSE
  - **PLLM:** 25
  - **PLLN:** 400
  - **PLLP:** DIV4
  - **PLLQ:** 4

These settings yield:
- **SYSCLK (System Clock):** 100MHz (HSE / PLLM * PLLN / PLLP = 25MHz / 25 * 400 / 4 = 100MHz)
- **AHB (HCLK):** 50MHz (SYSCLK / 2)
- **APB1 (PCLK1):** 25MHz (HCLK / 2)
- **APB2 (PCLK2):** 50MHz (HCLK / 1)

### Configuration Code Reference
```c
RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
RCC_OscInitStruct.PLL.PLLM = 25;
RCC_OscInitStruct.PLL.PLLN = 400;
RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
RCC_OscInitStruct.PLL.PLLQ = 4;
// SYSCLK = (HSE_VALUE / PLLM) * PLLN / PLLP = (25MHz / 25) * 400 / 4 = 100MHz

RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;    // HCLK = SYSCLK / 2 = 50MHz
RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;     // PCLK1 = HCLK / 2 = 25MHz
RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;     // PCLK2 = HCLK / 1 = 50MHz
```

### Optimization
- All main peripherals, tasks, and communication interfaces are configured for reliable operation at 100MHz core frequency.
- Flash latency and wait states are automatically set for safe access at this frequency.
- AHB bus operates at 50MHz, providing efficient memory and peripheral access.

### Notes
- The STM32F411CEU6 supports up to 100MHz system clock (see STM32 device datasheet).
- The Black Pill development board uses a 25MHz external crystal oscillator.
- If using different external crystals or changing PLL settings, ensure all timings and peripheral clock dividers are adjusted accordingly.
- For further details, see clock setup code in `Core/Src/main.c` and `system_stm32f4xx.c`.

---

## Key Parameters and PID Tuning

### PID Controller for Heading Lock

The robot uses a PID controller to maintain heading during movement (heading lock mode). The PID is adaptive and includes anti-windup, oscillation, and overshoot detection.

**Default PID Parameters (see `main.c`):**
```c
#define PID_KP 0.7f    // Proportional gain (default, adaptive in code)
#define PID_KI 0.02f   // Integral gain (low to prevent windup)
#define PID_KD 0.1f    // Derivative gain (damping, reduces overshoot)
#define PID_MAX_OUTPUT 18.0f  // Max correction output (%)
#define PID_INTEGRAL_MAX 20.0f  // Integral anti-windup limit
#define PID_RATE_LIMIT 2.0f  // Max change per update cycle
```
- **KP**: Increase for more aggressive/faster heading correction.
- **KD**: Increase to improve response to rapid heading changes (e.g., wheel slip at high speed).
- **KI**: Use with caution; too high can cause instability.
- **Adaptive Gains:** The code increases gains for small errors and reduces them if oscillation or overshoot is detected.

**Oscillation & Overshoot Detection:**  
- The PID logic automatically reduces gains if it detects oscillation or overshoot, improving stability.

### Other Key Parameters

- **BNO085 I2C Clock:** 400 kHz (Fast Mode, optimal for this sensor)
- **BNO085 Report Interval:** Default 100 Hz (`sh2_config.reportInterval_us = 10000`), recommended 50 Hz (`20000`) for lower CPU load.
- **SBUS Timeout:** 100 ms (motors stop if no valid SBUS frame within this time)
- **Watchdog Timeout:** ~5 seconds (adjustable in `MX_IWDG_Init`)
- **Encoder Polling:** 50 Hz (`osDelay(20)` in EncoderTask)
- **Display Update:** 5 Hz (`osDelay(200)` in DisplayTask)

---

## FRAM Storage and Recovery

The firmware uses the MB85RC256V FRAM module (via I2C1 interface) to **save key data for recovery**, including:
- **Heading lock value:** The last locked heading is saved so it can be restored after a power cycle or SBUS signal loss.
- **Other persistent settings:** Any additional configuration or calibration data can be stored for robust operation.

This ensures the robot can recover its heading lock and other critical parameters after resets, power loss, or communication failures. The FRAM provides reliable, non-volatile storage with virtually unlimited write endurance compared to traditional EEPROM emulation.

---

## Pin Assignment

| Function                | STM32 Pin | Peripheral    | Notes                           |
|------------------------|-----------|---------------|---------------------------------|
| **Communication**      |           |               |                                 |
| SBUS RX (inverted)     | PA3       | USART2_RX     | Requires signal inverter        |
| I2C1 SCL               | PB6       | I2C1_SCL      | SSD1306 + BNO085 + FRAM        |
| I2C1 SDA               | PB7       | I2C1_SDA      | SSD1306 + BNO085 + FRAM        |
| BNO085 INT             | PA8       | EXTI8         | Interrupt for sensor data       |
| **Motor Control**      |           |               |                                 |
| Motor 1A PWM           | PA8       | TIM1_CH1      | Right Front Motor               |
| Motor 1A IN1           | PA0       | GPIO_OUT      | Direction control               |
| Motor 1A IN2           | PA1       | GPIO_OUT      | Direction control               |
| Motor 1B PWM           | PA9       | TIM1_CH2      | Left Front Motor                |
| Motor 1B IN1           | PB15      | GPIO_OUT      | Direction control               |
| Motor 1B IN2           | PA15      | GPIO_OUT      | Direction control               |
| Motor 2A PWM           | PA11      | TIM1_CH4      | Back Motor                      |
| Motor 2A IN1           | PB0       | GPIO_OUT      | Direction control               |
| Motor 2A IN2           | PB1       | GPIO_OUT      | Direction control               |
| TB6612 #1 STBY         | PB12      | GPIO_OUT      | Motor driver 1 standby          |
| TB6612 #2 STBY         | PB13      | GPIO_OUT      | Motor driver 2 standby          |
| **User Interface**     |           |               |                                 |
| Encoder CLK            | PA4       | EXTI4         | Rotary encoder clock            |
| Encoder DT             | PA6       | GPIO_IN       | Rotary encoder data             |
| Encoder SW             | PA5       | EXTI5         | Rotary encoder button           |
| Status LED             | PC13      | GPIO_OUT      | Onboard LED indicator           |

---

## SBUS Channel Mapping

| Channel | Function      | Description                    |
|---------|--------------|--------------------------------|
| 2       | Left/Right   | X-axis movement (strafe)       |
| 4       | Rotation     | Z-axis rotation control        |
| 5       | Forward/Back | Y-axis movement                |
| 7       | Mode Select  | Automatic mode switching       |

**Channel 7 Mode Selection:**
- **MIN**: Sensor Display Mode (Idle)
- **MID**: Manual Movement Mode (Running)
- **MAX**: Heading Lock Mode (Running)

---

## Operating Modes

### 1. Sensor Display Mode (`MODE_DISPLAY_SENSORS`)
- **Purpose**: Monitor sensor data and system status
- **Features**: 
  - Real-time compass and robot heading display
  - SBUS channel monitoring
  - Encoder position tracking
  - System diagnostics
- **State**: Always in IDLE (motors stopped)

### 2. Manual Movement Mode (`MODE_MOVEMENT`)
- **Purpose**: Direct joystick control without heading assistance
- **Features**:
  - Full omnidirectional movement
  - Direct motor speed control
  - Manual rotation control
- **States**: IDLE (stopped) or RUNNING (active)

### 3. Heading Lock Mode (`MODE_MOVEMENT_HL`)
- **Purpose**: Movement with automatic heading correction
- **Features**:
  - PID-controlled heading lock system
  - Automatic oscillation and overshoot detection
  - Persistent heading storage in FRAM
  - Signal recovery with heading restoration
- **States**: IDLE (stopped) or RUNNING (active with heading lock)

---

## Program Logic Overview

### Task Structure (FreeRTOS)
- **StartDefaultTask**: Main robot logic, SBUS signal handling, mode/state changes, heading lock, and SBUS recovery.
- **StartCompassTask**: Handles BNO085 INT events, updates heading from IMU.
- **StartDisplayTask**: Updates OLED display with status, errors, and debug info.
- **StartEncoderTask**: Handles rotary encoder input for manual mode selection.

### Key Logic Flow

1. **Startup**
   - Initialize peripherals, drivers, and FreeRTOS.
   - Scan I2C bus and check BNO085 presence.
   - Initialize OLED, FRAM, and watchdog.

2. **SBUS Handling**
   - Receive SBUS frames via DMA.
   - Validate and unpack channels.
   - Detect signal loss and trigger recovery if needed.

3. **Mode & State Management**
   - Modes can be changed via SBUS channel 7 or encoder.
   - State toggles between IDLE and RUNNING.
   - Heading lock is enabled/disabled based on mode/state.

4. **Heading Lock & PID**
   - When enabled, calculates heading error and applies PID correction to motor speeds.
   - Adaptive PID gains, anti-windup, and rate limiting.
   - Oscillation and overshoot detection adjust gains dynamically.
   - Heading lock state is saved/restored to/from FRAM for robustness.

5. **Safety & Recovery**
   - Immediate motor stop on SBUS loss or error.
   - Automatic SBUS recovery attempts in main task context.
   - Watchdog timer ensures system resets on hang or fault.
   - Error handler flashes LED and resets MCU on critical errors.

6. **Display & User Interface**
   - OLED shows current mode, state, heading, SBUS status, and errors.
   - Encoder allows manual mode selection and confirmation.

---

## Project Structure

```
.
├── Core/           # Main application source code (Src/Inc)
├── Drivers/        # HAL and device drivers
├── FreeRTOS/       # RTOS configuration and source
├── I-CUBE-EE/      # Legacy configuration file (FRAM now used instead)
├── .ioc            # STM32CubeMX project file
├── README.md
└── ...
```

---

## Usage

- Use your RC transmitter to control the robot via SBUS.
- IMU data from the BNO085 is used for precise motion control and orientation feedback.
- The OLED display provides real-time status or debug information.
- The rotary encoder (wheel and button) is used for menu navigation and user input.
- FreeRTOS manages tasks such as motor control, sensor reading, SBUS decoding, and UI updates.

---

## Contributing

Pull requests are welcome! For major changes, please open an issue first to discuss your ideas.

---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.

---

## Acknowledgements

- [STMicroelectronics](https://www.st.com/)
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- Open-source libraries and contributors for SSD1306, BNO085, FreeRTOS, and MB85RC256V FRAM support

---

_Questions? Open an issue or start a discussion!_
