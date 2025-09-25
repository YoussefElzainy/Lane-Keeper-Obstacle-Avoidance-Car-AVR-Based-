# Lane Keeper & Obstacle Avoidance Car (AVR Based)

## 📌 Project Overview
This repository contains an **AVR-based autonomous car** that can **keep itself within a lane** and **avoid obstacles**.  
The system is implemented around the **ATmega32A** microcontroller and was developed as the final project for the Embedded Systems Diploma at **Telecom Egypt Training Sector**.

Key capabilities:
- **Lane Keeping** using IR sensors to detect lane markings  
- **Obstacle Avoidance** using ultrasonic sensors to detect and react to objects  
- Smooth motor control with H-bridge driver for navigation  

---

## 🛠️ Technologies & Tools
- **Microcontroller:** ATmega32A (AVR)  
- **Language:** Embedded C  
- **IDE / Toolchain:** Atmel Studio / Microchip Studio  
- **Simulation:** Proteus  
- **Hardware Components:**  
  - IR line sensors (lane detection)  
  - Ultrasonic sensor (obstacle detection)  
  - DC motors + wheels  
  - L293D motor driver  
  - Power supply module  

---

## ⚙️ System Features
- Lane detection using IR sensors  
- Obstacle detection and avoidance with ultrasonic sensors  
- PWM-based DC motor control for smooth movement  
- Modular and reusable code structure  
