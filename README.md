# 🤖 Line Maze Solver Robot using Arduino

### 🧠 Overview
The **Line Maze Solver Robot** is a fully **autonomous line-following robot (LFR)** designed to navigate mazes and tracks without human intervention.  
It uses an **Arduino UNO**, an **8-segment IR sensor array (RLS-08)**, and an **L298N motor driver** to detect and follow black lines on white surfaces using reflected infrared light.

This project demonstrates the fundamentals of **embedded systems**, **sensor-based decision-making**, and **autonomous navigation**.

---

### 🎯 Features
- 🚗 Fully autonomous line-following operation  
- 🧭 Real-time path detection using 8-segment IR array  
- ⚙️ Smooth motor control via PWM (L298N driver)  
- 🔋 Stable power management with buck converter  
- 🧠 Expandable to full maze-solving algorithms (Left/Right-hand rule)

---

### 🧩 Components Used

| Component | Quantity | Description |
|------------|-----------|-------------|
| Arduino UNO | 1 | Main controller board |
| IR Sensor Array (RLS-08) | 1 | 8-channel digital line sensor |
| L298N Motor Driver | 1 | Dual H-Bridge driver for motor control |
| N20 DC Motors | 2 | Left and right drive motors |
| Wheels + Chassis | 1 set | Base frame for the robot |
| Buck Converter | 1 | Converts battery voltage to stable 5V |
| Li-ion Battery | 1 | Power supply (7.4V–12V) |
| Jumper Wires | — | Connections between modules |

---

### ⚡ Circuit Description
- The **RLS-08 sensor module** detects line contrast by measuring infrared reflection.
- Each sensor output connects to a **digital input pin** on the Arduino.
- The Arduino processes these inputs and controls motor direction and speed using the **L298N driver**.
- The **buck converter** ensures a regulated **5V** supply to the Arduino and sensors.

---

### 🔌 Pin Connections

| Module | Pin | Arduino Pin |
|--------|-----|-------------|
| IR Sensor (OUT0–OUT7) | 8 outputs | D2–D9 |
| L298N IN1 | - | D10 |
| L298N IN2 | - | D11 |
| L298N IN3 | - | D12 |
| L298N IN4 | - | D13 |
| L298N ENA/ENB | - | D5, D6 (PWM) |
| Power | VCC | 5V (from buck converter) |
| GND | - | Common ground |

---

### 💻 Code Logic
The Arduino continuously reads signals from the IR sensors and makes movement decisions based on which sensors detect the black line.

#### Core Algorithm:
```cpp
if (center sensors detect line)
    moveForward();
else if (left sensors detect line)
    turnLeft();
else if (right sensors detect line)
    turnRight();
else
    stopMotors();
