
<img width="2048" height="588" alt="new_log" src="https://github.com/user-attachments/assets/ab09742d-f94e-4978-b8e5-fd3087f4f949" />

# Marut - STM32 F4XX Blackpill Based Fixed Wing + Multirotor FCU

**Marut** is an advanced Flight Control Unit (FCU) being developed at **AISSMS IOIT** in collaboration with **Team Aeroguardians**. It combines cutting-edge control algorithms, real-time telemetry, and autonomous navigation for both fixed-wing and quadrotor aircraft. Built for precision, reliability, and innovation, Marut aims to set a new standard in UAV flight control research and development.

This project is a collaborative initiative between **Team Aeroguardians** of the IoIT Drone Club and the **Catalyst Committee** to develop an indigenous Flight Control Unit (FCU). The aim is to advance drone and UAV development within the college by providing a **custom-designed, locally developed FCU right from the very core**.

## Team

* **Project Lead**: Aryan Basnet (TY ENTC)
* **Collaborators**:

  * Karan Tikoo (TY Comp)
  * Yash Tawar (SY ENTC)
  * Shreyas Fy (FY Comp)
  * Siddesh Vatvikar (SY ENTC)
  * Sharal Vishwakarma (SY ENTC)

## Project Phases

### Fixed Wing Phase

The Fixed Wing FCU development focuses on the core functionalities required for stable and controllable flight in fixed-wing aircraft.

* **ESC Driver Code**: Development of robust and efficient Electronic Speed Controller (ESC) driver code is paramount. This code will precisely control the speed and direction of the aircraft's motor(s), enabling thrust regulation for ascent, descent, and forward flight, including PWM and DShot protocols and motor synchronization.
* **SG90 & MG995 Servo Control Code**: Precise control code for hobby servos like SG90 and MG995 to manipulate control surfaces (ailerons, elevators, rudders) for pitch, roll, and yaw. Includes PWM generation, calibration, and feedback handling.
* **PPM & PWM RX Input Code**: Code to interpret signals from radio receivers (PPM & PWM), translating stick movements into FCU commands with error checking and fail-safe mechanisms.
* **BMP280/180 Barometer Interface**: Integration of barometric sensors for accurate altitude readings, including I2C communication, calibration, and conversion to altitude for telemetry and altitude hold.
* **Fixed-wing Control Surface Algorithms**: Algorithms translating desired maneuvers into control surface deflections using PID loops for stability and precision.
* **MAVLink Telemetry Integration**: Single-sided telemetry from the fixed-wing FCU to ground station software like QGC or Mission Planner for real-time monitoring.
* **9-Axis IMU Driver Code**: Development of driver code for a 9-axis IMU combining MPU6050 and QMC5883L for precise orientation sensing.

### Multirotor Phase

The Quad FCU development addresses multirotor-specific challenges.

* **Gyroscope Interface (MPU6050/MPU9250)**: Integration of IMU for angular rate data to detect and correct roll, pitch, and yaw using high-frequency readings and filtering.
* **3-input PID Stabilization Code**: Optimized PID system using gyroscope data to stabilize the quadcopter across all rotational axes with tuned gains.
* **Quad Control Surface Algorithms**: Algorithms converting desired maneuvers into differential thrust commands for each motor, coordinating with PID stabilization for stable and controllable flight.

## License

This project is **open for collaboration within the institution**.

For external collaborations, please contact the Catalyst Committee.

---
