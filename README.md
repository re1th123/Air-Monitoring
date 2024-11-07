STM32 FreeRTOS Project: EDF Scheduling with Priority Ceiling Protocol (PCP)
This project demonstrates how to set up tasks on an STM32 microcontroller using FreeRTOS with Earliest Deadline First (EDF) scheduling and the Priority Ceiling Protocol (PCP) to manage shared resources.

Table of Contents
Introduction
Project Setup
Implementation
Task Creation
EDF Scheduling
Priority Ceiling Protocol (PCP)
Dependencies
Building and Running
Testing
Introduction
This project is built to demonstrate real-time task scheduling on an STM32 microcontroller with FreeRTOS. It includes:

EDF Scheduling: Prioritizing tasks with the nearest deadlines.
PCP: Managing access to shared resources and preventing priority inversion.
Features
Task scheduling based on EDF.
Priority management for shared resources using PCP.
Integration with DHT11 temperature/humidity sensor.
OLED display to show sensor data.
Project Setup
Hardware Requirements

STM32 Microcontroller (e.g., STM32F401).
DHT11 temperature and humidity sensor.
OLED Display connected via I2C.
Software Requirements

STM32CubeIDE: For code development and debugging.
FreeRTOS: For task scheduling and real-time control.
Connections

Connect the DHT11 sensor to a GPIO pin on the STM32.
Connect the OLED display to the I2C pins of the STM32.
Implementation
Task Creation
Define each task function and create tasks with FreeRTOS. In this project, we have two main tasks:

Task 1: Reads sensor data from DHT11 and displays it on the OLED.
Task 2: Simulates another task that could represent additional functionality.
Tasks are created in main.c using FreeRTOS functions to set stack size, priority, and handles.

EDF Scheduling
To implement EDF scheduling, we create a separate EDF Scheduler Task that runs every 50 ms:

Each task has a predefined deadline.
The scheduler task checks each task’s remaining time to its deadline and adjusts priorities based on which task has the closest deadline.
Priority Ceiling Protocol (PCP)
PCP is used to manage access to the shared resource (e.g., resourceMutex):

PCP Lock: Before accessing the resource, each task locks it using a PCP lock function, which raises the task’s priority to the ceiling level.
PCP Unlock: After accessing the resource, the task unlocks it using a PCP unlock function, which restores the task’s original priority.
Each task uses PCP functions to control access to the shared resource.

Dependencies
FreeRTOS: Real-time operating system kernel for task management.
HAL Drivers: STM32 HAL (Hardware Abstraction Layer) drivers for peripheral management.
DHT11 Library (optional): Custom code to communicate with the DHT11 sensor.
SSD1306 Library (optional): Library for controlling the OLED display.
Building and Running
Open the Project in STM32CubeIDE:

Clone the project or set it up in STM32CubeIDE.
Configure peripherals (GPIO, I2C) as needed in the .ioc configuration file.
Build the Project:

Select Build Project in STM32CubeIDE to compile the code.
Ensure there are no errors in the build output.
Flash the STM32:

Connect the STM32 microcontroller to your computer.
Use the Debug or Run option in STM32CubeIDE to flash the code onto the microcontroller.
Run:

The tasks will run automatically. Task 1 will read data from the sensor and display it on the OLED every 2000 ms. Task 2 runs at a 1000 ms interval.
Testing
Verify EDF Scheduling:

Observe the task execution order in STM32CubeIDE’s FreeRTOS viewer to confirm that the task with the nearest deadline runs first.
Verify Priority Ceiling Protocol:

Ensure that when task1 or task2 locks the resource, it maintains control until the unlock function is called, preventing priority inversion.
Sensor and Display Output:

Check that the OLED displays updated temperature and humidity readings every 2000 ms.
