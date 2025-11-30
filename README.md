# Quadruped Robot Dog 

This project is a design and implementation of a 4-legged (quadruped) robot dog using 3D printed parts and Arduino control. The core of the project involves calculating **Inverse Kinematics (IK)** to map Cartesian coordinates $(x, y)$ in space to specific servo angles for the leg joints.

![Robot Dog Prototype](Demo_Pics/Assembly.jpeg)

## 🔧 Hardware Specs
* **Microcontroller:** Arduino Uno / Nano
* **Servo Driver:** PCA9685 16-Channel 12-bit PWM Driver (I2C)
* **Actuators:** 8x MG90S Metal Gear Micro Servos (2 per leg)
* **Power:** External 5V High-Current Supply
* **Body:** Custom 3D Printed PLA chassis and linkage system

## 📐 The Math: Inverse Kinematics
To make the robot walk, we cannot simply tell the servos to move to random angles. We must define a path for the foot (end-effector) to follow and calculate the joint angles required to reach that point.

I derived the geometry using the **Law of Cosines** to solve the 2-Link planar arm problem for each leg.

![Inverse Kinematics Derivation](Demo_Pics/Inverse_Kinematics.jpg)

### 1. Leg Geometry
Given a target height $C$ (vertical distance from hip to ground) and leg segment lengths:
* $L_1$ (Thigh) = 12.2 cm
* $L_2$ (Shin) = 11.5 cm

We calculate the hip angle ($A$) and knee angle ($B$) using the Law of Cosines:

$$\cos(A) = \frac{L_1^2 + C^2 - L_2^2}{2 \cdot L_1 \cdot C}$$

$$\cos(B) = \frac{L_1^2 + L_2^2 - C^2}{2 \cdot L_1 \cdot L_2}$$

### 2. Trajectory Planning
To simulate a walking gait, the foot follows a semi-circular path defined by the equation of a circle:

$$y = \sqrt{r^2 - x^2}$$

Where:
* $x$ is the horizontal step distance.
* $y$ is the vertical step height (lift).
* $r$ is the radius of the step.

As the foot moves horizontally by $\delta x$, the inverse kinematics engine recalculates the new required angles $\theta_{hip}$ and $\theta_{knee}$ in real-time to keep the foot on the trajectory.

## Code Overview
The software utilizes the `Adafruit_PWMServoDriver` library to communicate with the PCA9685 via I2C. This allows for smooth control of all 8 servos simultaneously using only 2 pins on the Arduino.

### dog_with_pot_control
This script is designed for servo calibration and testing:

Uses a potentiometer to manually control leg motion.

Helps determine the minimum and maximum travel angles for each servo.

Useful for load testing to evaluate how much weight the robot can carry (limited by servo quality).

Compatible with different servo models, but ensure you update:

Servo frequency in the driver settings.

Min/max angle values in the code.

-Think of this as the diagnostic tool — it lets you experimentally verify servo limits before moving on to gait control.

### leg_movement
This script provides the inverse kinematics (IK) framework for robot locomotion:

Implements trajectory planning for walking gaits.

Calculates hip and knee angles from Cartesian foot positions in real time.

Requires customization based on:

3D printed part tolerances (geometry may vary).

Servo differences (torque, speed, travel range).

-Best practice: first run dog_with_pot_control to obtain calibrated parameters, then integrate those values into leg_movement for accurate IK execution.

## Future Improvements
* Implementation of full Trot and Creep gaits.
* Integration of an IMU (Gyroscope) for self-balancing.
* Wireless control via Bluetooth/RF.
