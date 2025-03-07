# CameraVision_DC_Motor_Servo_Control
 This project uses computer vision with Python and OpenCV to analyze real-time camera input. It detects specific colors and computes line angles to determine object position and orientation. Based on the detected color (red, green, or yellow) and the calculated angle, commands are sent via serial communication to an ESP32. The ESP32 then controls a DC motor (via an H-bridge) and a servo motor to adjust speed and position, achieving vision-driven motor control.

This project uses computer vision (Python & OpenCV) to control a DC motor and a servo via an ESP32 through serial communication. The system is divided into two modules:

- **Servo Control:**  
  - **Python Code:** Captures images, computes a line’s angle, and sends the angle via serial.  
  - **ESP32 Code:** Receives the angle command and moves a servo accordingly.

- **Motor Control:**  
  - **Python Code:** Detects colors (red, green, yellow) in real time and sends a corresponding speed value via serial.  
  - **ESP32 Code:** Receives the speed value and controls a DC motor (using an H-bridge) via PWM.

## Files

- `servo_control_python.py`  
  Python script that captures camera input, calculates a line angle, and sends the angle over serial.

- `servo_control_esp32.ino`  
  ESP32 Arduino sketch that receives an angle and controls a servo (using ESP32Servo library) on a designated pin.

- `motor_control_python.py`  
  Python script that detects predominant colors and sends a speed value based on:  
  - Red: 255  
  - Green: 180  
  - Yellow: 110  
  - None: 0

- `motor_control_esp32.ino`  
  ESP32 Arduino sketch that receives the speed value and controls a DC motor through an H-bridge. The code uses PWM on an ENA pin (e.g., pin 32) along with two digital pins for motor direction (e.g., IN1 on pin 25 and IN2 on pin 26).

## Requirements

- **Hardware:**
  - ESP32 development board.
  - DC motor with H-bridge.
  - Servo motor.
  - Webcam for capturing images.
  - PC to run the Python scripts.

- **Software:**
  - Python 3 with:
    - OpenCV (`opencv-python`)
    - NumPy
    - PySerial
  - Arduino IDE with the [ESP32Servo](https://github.com/madhephaestus/ESP32Servo) library installed.

## Setup & Usage

1. **Install Python Dependencies:**
   ```bash
   pip install opencv-python numpy pyserial
