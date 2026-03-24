# STM32 UART Communication Protocol (MMCP)

This project implements a modular communication protocol (Multi-Controller Communication Protocol - MMCP) designed for efficient data exchange between STM32 microcontrollers. It follows a simplified 7-layer OSI model to ensure scalability and clear separation of concerns.

## 🚀 Key Features
* **Non-Blocking Architecture:** All UART operations are implemented using Interrupts and DMA to minimize CPU load.
* **Layered Design:** Strict separation of layers (L1 Physical to L7 Application) for modular development.
* **Data Integrity:** L2 implements a CRC-8 check (Polynomial 0xCF) to ensure reliable data transmission.
* **Real-Time Visualization:** Integrated WS2812 LED driver using PWM and DMA for high-precision timing without CPU stalling.

## 🛠 Technical Implementation

### Communication Stack (mmcp.c)
The protocol handles packet encapsulation and decapsulation across the stack:
* **L1 (Physical):** Frame detection with SOF/EOF.
* **L2 (Data Link):** CRC-8 calculation and validation.
* **L3 (Network):** Address-based routing and hop-count management.
* **L7 (Application):** Dispatcher for various application process numbers (APNR) such as status requests and internal logic.

### Hardware Integration (ws2812.c)
To provide visual feedback of the system state, I implemented a WS2812 driver. 
* **Challenge:** WS2812 requires strict 800kHz timing (1.25µs per bit).
* **Solution:** Used a Timer in PWM mode combined with DMA. This allows the CPU to continue processing communication logic while the hardware handles the signal generation.

## 📂 Project Structure
* `main.c`: Central state machine and event handling.
* `mmcp.c`: Implementation of the OSI-based communication layers.
* `ws2812.c`: Hardware-near driver for LED visualization.
* `init.c`: Low-level peripheral configuration (Clock, DMA, UART, Timer).

## 💡 About this Project
This project was developed during my Electrical Engineering studies (6th semester) to bridge the gap between theoretical communication models and hardware-near C programming. It demonstrates my ability to work with professional embedded tools and handle real-time constraints.
