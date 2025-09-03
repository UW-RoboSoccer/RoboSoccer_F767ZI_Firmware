# RoboSoccer F767ZI Firmware

Firmware for the University of Waterloo RoboSoccer humanoid robot, built on the **STM32F767ZI Nucleo** platform.  
This firmware provides real-time control of sensors and actuators, and serves as the embedded foundation for the RoboCup 2026 competition.

---

## Features

- **RTOS-based architecture** using FreeRTOS with dedicated tasks:
  - `ImuTask` – 2.5 ms loop for BNO085 IMU fusion data.
  - `MotorTask` – synchronized servo control/feedback for 20+ motors.
  - `AnalogTask` – ADC sampling of FSR sensors with averaging.
  - `GeneralTask0` – system-level events and housekeeping.
- **BNO085 IMU driver** (SPI-DMA with CEVA-style state machine).
- **Servo communication stack** for STS3215 servos over UART (DMA or interrupt driven).
- **Force sensor array** using FSR03CE sensors and STM32 ADC averaging macros.
- **High-speed communication support** (USB HS via external ULPI PHY).
- Modular driver design with clean separation of hardware abstraction and application logic.

---

## Hardware

- STM32F767ZI Nucleo board  
- BNO085 IMU (SPI)  
- STS3215 smart servos (UART bus)  
- FSR03CE force sensors (ADC inputs)  
- USB3300 ULPI PHY (USB HS)  
- Custom PCBs for interfacing sensors and actuators  

---

## Project Structure

RoboSoccer_F767ZI_Firmware/
├── Core/
│ ├── Src/ # Application code
│ ├── Inc/ # Headers for tasks, drivers, utilities
├── Drivers/ # STM32 HAL and custom peripheral drivers
├── FreeRTOS/ # RTOS configuration and CMSIS wrappers
├── Middlewares/ # (e.g., SH2 library for BNO085)
└── README.md


---

## Build & Flash

1. **Toolchain**: [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) (recommended), or `arm-none-eabi-gcc` + Make.  
2. Open the project in CubeIDE, configure the target as `STM32F767ZI`.  
3. Connect the Nucleo board via ST-Link.  
4. Build (`Ctrl+B`) and flash (`Run` or `Debug`).  

---

## Contributing

1. Fork the repository.  
2. Create a feature branch (`git checkout -b feature/my-feature`).  
3. Commit changes (`git commit -m "Add my feature"`).  
4. Push branch (`git push origin feature/my-feature`).  
5. Open a Pull Request.  
