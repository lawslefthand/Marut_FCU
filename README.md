<img width="580" height="472" alt="image" src="https://github.com/user-attachments/assets/07e5f51a-5438-463c-ba79-83d2b13f661f" />


# Indigenous Flight Control Unit (FCU) Development
**A Collaborative Initiative Between Team Aeroguardians and the Catalyst Committee**

This project outlines a collaborative initiative between **Team Aeroguardians** of the IoIT Drone Club and the **Catalyst Committee** to develop an indigenous Flight Control Unit (FCU).  
The aim is to advance drone and UAV development within the college by providing a **custom-designed, locally manufactured FCU**.  

This collaboration leverages:
- **Team Aeroguardians**: Expertise in drone technology and R&D infrastructure.  
- **Catalyst Committee**: Developmental, organizational, and project management support.  

---

## Introduction

### The Catalyst Committee
The Catalyst Committee is a strategic initiative guided by:
- **Dr. Venkat Ghodke**, Training and Placement Officer  
- **Dr. P.B. Mane**, Principal  

**Composition**: Select students with exceptional skills in hardware and software.  

**Primary Mandate**:
- Elevate the college's reputation for technical excellence.
- Facilitate participation in hackathons and competitions.
- Execute impactful industry-sponsored projects.

### Team Aero Guardians
**Team Aero Guardians** is the premier R&D team of the IoIT Drone Club, facilitated by **Mr. Rahul Jadhav**, Professor of Electronics and Telecommunications.  

**Achievements & Focus**:
- Victory at **Maha Hackathon Challenge 2025**.
- Competing at **NIDAR 2025** (National Innovation Challenge for Drone Application and Research).
- Collaboration with **Government of Maharashtra** on Search and Rescue & Disaster (SARD) operations.

---

## FCU Development Plan

The FCU development is structured into **four main modules**:  

> Progression to the next subsection is only done after successful testing.

---

### I. Fixed Wing FCU
Core functionalities for stable and controllable flight in fixed-wing aircraft:

- **ESC Driver Code**  
  - Control motor speed and direction.
  - Support protocols like PWM, DShot.
  - Ensure motor synchronization.

- **SG90 & MG995 Servo Control Code**  
  - Control surfaces: ailerons, elevators, rudders.
  - Handle PWM signal generation, calibration, and feedback.

- **PPM & PWM RX Input Code**  
  - Interpret manual control from radio receivers.
  - Translate stick movements to FCU commands.
  - Implement fail-safes for signal loss.

- **BMP280/180 Barometer Interface**  
  - Accurate altitude readings via I2C.
  - Calibrate pressure data and convert to altitude.

- **Fixed-wing Control Surface Algorithms**  
  - Implement PID control loops for pitch, roll, yaw.
  - Translate desired maneuvers into surface deflections.

---

### II. Quad FCU
Specialized development for quadrotor aircraft:

- **Gyroscope Interface (MPU6050 / MPU9250)**  
  - Acquire angular rate data for stability.
  - Configure sensor, read raw data, apply filters.

- **3-input PID Stabilization Code**  
  - Stabilize roll, pitch, and yaw axes.
  - Optimize motor thrust adjustments.

- **Quad Control Surface Algorithms**  
  - Convert desired maneuvers into differential motor thrust.
  - Work alongside PID for stable flight.

---

### III. Telemetry Subsystems
Enable real-time communication between aircraft and ground station:

- **Basic Wireless Telemetry Code (MAVLink)**  
  - Package sensor data and flight status.
  - Standardized lightweight messaging protocol.

- **Wireless Telemetry Module Integration**  
  - Examples: 3DR radio, ESP-NOW, LoRa.
  - Hardware interfacing via UART.
  - Ensure reliable data transmission.

- **Ground Station Software Design**  
  - Receive and decode MAVLink data.
  - Display flight parameters: altitude, GPS coordinates, battery.
  - Graphical interface for mission planning and remote control overrides.

---

### IV. Autonomous Subsystems
Provide autonomous mission capabilities:

- **GPS Module Interface (Neo M8N)**  
  - Extract latitude, longitude, altitude, velocity.
  - Communicate via UART or I2C.

- **NMEA / UBX Coordinate Parsing Code**  
  - Parse GPS data formats.
  - Extract meaningful positional information.

- **Waypoint-to-Waypoint Navigation Code**  
  - Navigate autonomously between predefined waypoints.
  - Calculate bearing and distance to next waypoint.
  - Use proportional or L1 guidance algorithms.

- **Geofencing Code**  
  - Define virtual operational boundaries.
  - Trigger safety actions if aircraft leaves geofence.

- **Intelligent Decision-Making Code**  
  - Obstacle avoidance, dynamic path planning.
  - Integrate sensor fusion, machine learning, or rule-based logic.

---

## License
This project is **open for collaboration within the institution**.  
For external collaborations, please contact the Catalyst Committee.

---

## Contributors
- **Team Aeroguardians** – IoIT Drone Club  
- **Catalyst Committee** – Institutional project initiative  

---

