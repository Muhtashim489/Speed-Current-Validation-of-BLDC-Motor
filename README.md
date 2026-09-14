# Speed-Current-Validation-of-BLDC-Motor
1- Objectives
The target is about to validate the required speed as close as much of the BLDC motor and also determine how much motor is drawing the current during this process. By seeking this, the primary goal is to understand motor dynamics, real applications for drones and enhanced hardware improvement.

2- Introduction
This project focuses on designing a system for validating and monitoring the RPM (rotations per minute) of a BLDC motor using the TM4C123GH6PM microcontroller, with real time display of speed on a 16x2 LCD and plot the graph on excel using serial communication.

3- Components description
The required materials for managing the whole procedure and develop such systems are as following
a) A2212 BLDC Motor
b) 30 A Electronic Speed Controller
c) 10K potentiometer
d) Drone arm and propeller 
e) Fuse holder with 15A fuse
f) 16x2 LCD Display
g) TM4C123GH6PM (Tiva C Series)

4- Project Process
The TM4C123GH6PM microcontroller processes sensor inputs to calculate and display motor speed. The ACS712 current sensor monitors current draw for load detection, while an optocoupler generates pulse signals to measure RPM. A potentiometer adjusts motor speed by varying voltage, allowing users to observe its effect on performance. The system maps standard speed values against input voltage for comparison with measured RPM.

5- Steps to run the project in flowchart context
	Reload and downloading the code
	Press the reset button
	Voltage will be varied by using potentiometer
	The LCD will display RPM and Current value.
	When adc is in range between 100 and 4095, the motor will start to rotate.
	Optocoupler will calculate number of edges and determine RPM.
	Current sensor detects how much motor draws current.
	Data transmission of rpm and current would be done by using UART.
	Data will plot on microsoft excel sheet.
