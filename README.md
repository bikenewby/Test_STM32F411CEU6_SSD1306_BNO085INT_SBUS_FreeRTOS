# STM32F411CEU6 Omniwheels Robot (3-Wheel Configuration)
This repository contains the firmware and source code for an omniwheels robot with a 3-wheel configuration, powered by an STM32F411CEU6 microcontroller. The project uses STM32CubeIDE and FreeRTOS, and features integration with an SSD1306 OLED display, BNO085 IMU sensor, SBUS RC receiver, and dual TB6612FNG motor drivers.

## Features
- **Omniwheels drive logic** for holonomic 3-wheel motion
- **SSD1306 OLED display** for real-time status and debug information
- **BNO085 IMU** for orientation and motion sensing (connected via I2C1)
- **SBUS receiver** interface for RC signal decoding
- **FreeRTOS** for real-time multitasking and robust scheduling
- **Rotary Encoder** (rotary wheel and push button) for user input
- Modular codebase for easy adaptation and feature expansion

## Hardware Requirements
- STM32F411CEU6 "Black Pill" or compatible board
- 3 omniwheels with appropriate motor drivers (e.g., TB6612FNG x2)
- SSD1306 OLED display (I2C)
- BNO085 IMU sensor (I2C)
- SBUS receiver (e.g., FrSky or compatible RC system)
- Rotary encoder (with wheel and push button, for menu/input)
- Power supply suitable for your motors and controller
- ST-Link V2 or compatible programmer/debugger

## STM32F411CEU6 Pin Assignment Table

| Function                  | STM32 Pin   | Notes / Peripheral                 |
|---------------------------|-------------|------------------------------------|
| **SBUS RX (from inverter)** | PA3         | USART2_RX, works for SBUS          |
| **I2C1_SCL**              | PB6         | I2C1 (SSD1306, BNO085)             |
| **I2C1_SDA**              | PB7         | I2C1 (SSD1306, BNO085)             |
| **I2C2_SCL**              | PB10        | Additional I2C (no PA8 conflict)   |
| **I2C2_SDA**              | PB11        | Additional I2C                     |
| **Onboard LED**           | PC13        | User indicator                     |
| **Encoder CLK**           | PA4         | EXTI interrupt input (rotary wheel)|
| **Encoder DT**            | PA6         | GPIO input (rotary wheel)          |
| **Encoder SW**            | PA5         | EXTI interrupt input (push button) |
| **Motor 1A PWM**          | PA8         | TIM1_CH1 (PWM)                     |
| **Motor 1A IN1**          | PA0         | GPIO output                        |
| **Motor 1A IN2**          | PA1         | GPIO output                        |
| **Motor 1B PWM**          | PA9         | TIM1_CH2 (PWM)                     |
| **Motor 1B IN1**          | PA2         | GPIO output                        |
| **Motor 1B IN2**          | PA15        | GPIO output                        |
| **TB6612 #1 STBY**        | PB12        | Motor driver 1 standby             |
| **Motor 2A PWM**          | PA11        | TIM1_CH4 (PWM)                     |
| **Motor 2A IN1**          | PB0         | GPIO output                        |
| **Motor 2A IN2**          | PB1         | GPIO output                        |
| **Motor 2B PWM**          | PA7         | TIM3_CH2 (PWM)                     |
| **Motor 2B IN1**          | PB14        | GPIO output                        |
| **Motor 2B IN2**          | PB15        | GPIO output                        |
| **TB6612 #2 STBY**        | PB13        | Motor driver 2 standby             |
| **BNO085 INT**            | [PA8/PB8]   | If needed, assign free INT pin     |
| **Display RESET**         | [Any GPIO]  | If needed                          |

**Note:**  
- Both the SSD1306 OLED display and BNO085 IMU sensor are connected to the MCU via I2C1 (PB6: SCL, PB7: SDA).
- The rotary encoder is used exclusively as a user interface input device (rotary wheel and push button).

## Getting Started

### 1. Clone This Repository
```bash
git clone https://github.com/bikenewby/Test_STM32F411CEU6_SSD1306_BNO085INT_SBUS_FreeRTOS.git
cd Test_STM32F411CEU6_SSD1306_BNO085INT_SBUS_FreeRTOS
```

### 2. Open in STM32CubeIDE
- Launch STM32CubeIDE
- Go to **File → Open Projects from File System**
- Select this repository folder to import the project

### 3. Connect Hardware
Wire each module according to the pin assignment table above. Ensure all power and ground connections are secure and compatible with your peripherals (motors, sensors, display, etc.).

### 4. Build and Flash
- Click **Build** in STM32CubeIDE
- Connect your board via ST-Link
- Click **Debug** or **Run** to program the device

## Project Structure

```
.
├── Core/           # Main application source code (Src/Inc)
├── Drivers/        # HAL and device drivers
├── FreeRTOS/       # RTOS configuration and source
├── .ioc            # STM32CubeMX project file
├── README.md
└── ...
```

## Usage
- Use your RC transmitter to control the robot via SBUS.
- IMU data from the BNO085 is used for precise motion control and orientation feedback.
- The OLED display provides real-time status or debug information.
- The rotary encoder (wheel and button) is used for menu navigation and user input.
- FreeRTOS manages tasks such as motor control, sensor reading, SBUS decoding, and UI updates.

## Contributing
Pull requests are welcome! For major changes, please open an issue first to discuss your ideas.

## License
This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.

## Acknowledgements
- [STMicroelectronics](https://www.st.com/)
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- Open-source libraries and contributors for SSD1306, BNO085, and FreeRTOS support

---

_Questions? Open an issue or start a discussion!_
