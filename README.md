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

- `servo.py`  
  Python script that captures camera input, calculates a line angle, and sends the angle over serial.

- `servo.ino`  
  ESP32 Arduino sketch that receives an angle and controls a servo (using ESP32Servo library) on a designated pin.

- `motordc.py`  
  Python script that detects predominant colors and sends a speed value based on:  
  - Red: 255  
  - Green: 210  
  - Yellow: 180  
  - None: 0

- `motorDC.ino`  
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
   ```

2. **Run codes**

* Servo Control

This module controls a servo using an ESP32 and a Python script.

A. Upload ESP32 Code

Open and upload **`servo_control_esp32.ino`** to the ESP32.

B. Run the Python Script

Run **`servo_control_python.py`** to test servo control via angle detection.



* Motor Control

This module controls a DC motor (via an H-bridge) using an ESP32 and a Python script.

A. Upload ESP32 Code

Open and upload **`motor_control_esp32.ino`** to the ESP32.

B. Run the Python Script

Run **`motor_control_python.py`** to test motor control via color detection.

- Important Notes

Make sure that the **serial port** and **baud rate** in the .py code matches those used in the .ino.



   
## How It Works

### Servo Control
- **Image Processing:**  
  The Python script captures frames from the webcam, processes them (e.g., edge detection, Hough Transform) to calculate the angle of a detected line.
- **Command Transmission:**  
  The computed angle is sent via serial communication to the ESP32.
- **Servo Adjustment:**  
  The ESP32 receives the angle and adjusts the servo position accordingly using the ESP32Servo library.

### Motor Control
- **Color Detection:**  
  The Python script converts each frame to the HSV color space and applies color masks to detect red, green, or yellow.
- **Speed Determination:**  
  Depending on the predominant color:
  - **Red:** Speed value of 255  
  - **Green:** Speed value of 180  
  - **Yellow:** Speed value of 110  
  - **None:** Speed value of 0
- **Command Transmission:**  
  The chosen speed value is sent via serial to the ESP32.
- **Motor Speed Control:**  
  The ESP32 receives the speed value and controls a DC motor through an H-bridge using PWM.

