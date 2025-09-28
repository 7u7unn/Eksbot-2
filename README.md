# Autonomous Differential Drive Robot Navigation Stack

This repository contains the step-by-step development of an autonomous navigation stack for a differential drive robot.  
The project starts from collecting encoder and IMU raw data, then gradually builds up to odometry fusion, calibration, and autonomous navigation features.

---

## Table of Contents
- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Development Roadmap](#development-roadmap)
- [Modules](#modules)
  - [Motor Control](#motor-control)
  - [Odometry](#odometry)
    - [Encoder Odometry](#encoder-odometry)
    - [IMU Processing](#imu-processing)
    - [Encoder + IMU Fusion](#encoder--imu-fusion)
    - [Calibration](#calibration)
  - [Extras](#extras)
    - [Line Alignment](#line-alignment)
    - [Sensor Value Reading](#sensor-value-reading)
- [Installation](#installation)
- [Usage](#usage)
- [Hardware Setup](#hardware-setup)
- [Contributing](#contributing)
- [License](#license)

---

## Overview
Brief explanation of what the navigation stack is, its purpose, and the long-term goal of the project.
![Workchart](figure/workchart.png)

Also here is some note i found out about esp32 wroom that for me (whose new to esp32) important
- [PINOUT Caution](https://lastminuteengineers.com/esp32-wroom-32-pinout-reference/)
from this web, i found that some pins cant be used as output especially 34-39 (input only)

- Safe  pin:
    32, 33, 25, 26, 27, 14, 12, 13, 4, 16, 17, 18, 19, 21, 22, 23
- ADC1:
    36, 39, 34, 35, 32, 33
---

### Pin Usage
1. **Motor Driver** 
ENA, ENB, IN1, IN2, IN3, IN4 = 22, 23, 21, 19, 17, 18

2. **MUX**
S0,S1,S2,S3,SIG = 25, 33, 32, 26, 34

3. **IMU**
SCL, SDA = 12,13

4. **Encoder**
A1, B1, A2, B2 = 35, 36, 14, 16



## Development Roadmap
1. **Parameter Calibration** – Extract internal parameters from encoders 
    - For motor and encoder, first use gearbox.ino to get gearbox ratio for each motor (one of the pair of hall sensor in encoder must be put interrupt pin for better reading)
    - From GearRatio we can easly get ticks to cm ratio
    - 
2. **IMU Calibration** – Collect raw IMU data and process bias/noise  
3. **Encoder Odometry** – Implement differential drive odometry  
4. **IMU Processed Data** – Clean and filter IMU data  
5. **Encoder + IMU Fusion** – Fuse encoder odometry and IMU for better localization  
6. **Calibration Module** – Fine-tune and test odometry results  
7. **Motor Control** – Integrate closed-loop control for precise movement  
8. **Line Alignment & Sensor Reading (Extras)** – Add line alignment and other utility features  
9. **Autonomous Navigation** – Full integration of all modules

---

## Modules

### Motor Control
- Description of motor driver, PID, and control strategy.

### Odometry

#### Encoder Odometry
- How raw encoder data is converted to linear and angular displacement.  
- Parameter calibration details.

#### IMU Processing
- How raw IMU data (gyroscope, accelerometer) is processed and filtered.  
- IMU calibration steps.

#### Encoder + IMU Fusion
- Fusion algorithm used (e.g., complementary filter, EKF).  
- Expected improvements over individual sources.

#### Calibration
- Calibration process for both encoder and IMU parameters.  
- Testing procedures.

### Extras

#### Line Alignment
- Method for aligning the robot using line sensors.  
- Example applications.

#### Sensor Value Reading
- Sensor integration for navigation or obstacle avoidance.  
- Data visualization methods.


