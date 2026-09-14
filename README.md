# Speed & Current Validation of BLDC Motor

## 1. Objectives
The main goal of this project is to validate the actual RPM of a BLDC motor against target speeds while monitoring real-time current draw. 

**Key Objectives:**
* Analyze BLDC motor dynamics under varying loads.
* Validate performance characteristics for drone propulsion systems.
* Gather empirical data for future hardware optimizations.

---

## 2. Introduction
This project details the design and implementation of an embedded monitoring and validation system for a BLDC motor using the **TM4C123GH6PM (Tiva C)** microcontroller. 

The system reads speed control inputs, measures actual rotational speed and current consumption, displays metrics live on a 16x2 LCD, and streams telemetry data via UART for real-time graphical plotting in Microsoft Excel.

---

## 3. Materials

| Component | Description / Specification |
| :--- | :--- |
| **Microcontroller** | TM4C123GH6PM (Tiva C Series) |
| **Motor** | A2212 BLDC Motor |
| **ESC** | 30A Electronic Speed Controller |
| **Current Sensor** | ACS712 Current Sensor Module |
| **Speed Sensor** | Optocoupler (Optical Tachometer setup) |
| **Display** | 16x2 Character LCD |
| **Input Control** | 10kΩ Potentiometer |
| **Protection** | Fuse Holder with 15A Fuse |
| **Mechanical Test Rig** | Drone Arm & Propeller assembly |

---

## 4. System Architecture & Operation
The **TM4C123GH6PM** acts as the central processing unit:

1. **Speed Command:** A 10kΩ potentiometer acts as an analog input to set the motor throttle.
2. **Current Monitoring:** An **ACS712 sensor** continuously tracks line current to evaluate electrical load.
3. **RPM Feedback:** An **optocoupler** detects rotor passes, generating digital pulse edges to calculate rotational speed.
4. **Telemetry & Output:** Data is sent to the 16x2 LCD for live display and transmitted via UART to Microsoft Excel for visual plotting and logged comparison against theoretical speed mappings.
