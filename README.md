# Autonomous Differential Drive Robot Navigation Stack

This repository contains the step-by-step development of an autonomous navigation stack for a differential drive robot.  
The project starts from collecting encoder and IMU raw data, then gradually builds up to odometry fusion, calibration, and autonomous navigation features.



## Dev Flow Breakdown : 

<img src="figure/workchart.png" width="600">


---
## Development Roadmap
1. **Parameter Calibration** – Extract internal parameters from encoders 
    - For motor and encoder, first use gearbox.ino to get gearbox ratio for each motor (one of the pair of hall sensor in encoder must be put interrupt pin for better reading)
    - From GearRatio we can easly get ticks-to-cm ratio
2. **IMU Calibration** – Collect raw IMU data and process bias/noise  
3. **Encoder Odometry** – Implement differential drive odometry  
    - add gearbox ratio to this part
4. **IMU Processed Data** – Clean and filter IMU data  
5. **Encoder + IMU Fusion** – Fuse encoder odometry and IMU for better localization  
6. **Calibration Module** – Fine-tune and test odometry results  
7. **Motor Control** – Integrate closed-loop control for precise movement  
8. **Line Alignment & Sensor Reading (Extras)** – Add line alignment and other utility features  
9. **Autonomous Navigation** – Full integration of all modules

---
## Electrical Notes
Here is some notes i found out about esp32 wroom that for me (whose new to esp32) important
- [PINOUT Caution](https://lastminuteengineers.com/esp32-wroom-32-pinout-reference/)
from this web, i found that some pins cant be used as output especially 34-39 (input only)

- Safe pins:
    **4, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33**
- ADC1:
    **32, 33, 34, 35, 36, 39**

- Sisa:
    4, 27, 39

### Pin Usage
1. **IMU**
SCL, SDA = **12, 13**

2. **Encoder**
A1, B1, A2, B2 = **14, 16, 35, 36**

3. **Motor Driver** 
ENA, ENB, IN1, IN2, IN3, IN4 = **17, 18, 19, 21, 22, 23**

4. **MUX**
S0, S1, S2, S3, SIG = **25, 26, 32, 33, 34**
---


