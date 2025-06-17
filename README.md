# SVK Fox Robot

SVK Fox is an enhanced obstacle avoidance and line-following robot developed by SVK Robotics. It combines ultrasonic sensing and an 8-sensor IR array (IR8) for precise line tracking and obstacle detection.

![SVK Fox Robot](https://svkrobotics.com/uploads/items/2025-06-04-fox_6-Photoroom.jpg)

---

## Overview

This robot uses a PID control algorithm to follow lines accurately while continuously monitoring distance with an ultrasonic sensor to detect and stop for obstacles.

- **Line Following:** Uses an 8-sensor IR array (SVKIR8) for detecting black lines and PID control for smooth navigation.
- **Obstacle Avoidance:** Ultrasonic sensor measures distance ahead and stops the robot if an object is detected within a set range.

---

## Features

- PID-based line following for smooth, responsive control.
- Adjustable PID parameters (Kp, Ki, Kd) for tuning behavior.
- Configurable maximum speed and obstacle detection distance.
- Integration with the [SVK IR8 sensor library](https://github.com/SVKROBOTICS/IR_8).

---

## Usage

1. **Configure Parameters:**

   - Set `MAX_SPEED` to your desired maximum motor speed.
   - Adjust PID constants (`Kp`, `Ki`, `Kd`) for optimal line following.
   - Define `maxDistance` (in cm) to set how close the robot can get to an obstacle before stopping.

2. **Hardware Setup:**

   - Connect the 8 IR sensors via a multiplexer to pins 10, 9, 8, and A0.
   - Connect motors to pins 2, 3, 4, 6, 5, and 7 as specified.
   - Connect ultrasonic sensor to pins A3 (trig) and A1 (echo).

3. **Calibration:**

   The robot calibrates the IR sensors on startup by sampling sensor values 300 times. Make sure to place the robot on a surface with a clear line for accurate calibration.

4. **Run the Code:**

   Upload the code to your Arduino-compatible board. The robot will start following lines and stop when an obstacle is detected.

---

## How it Works

- The ultrasonic sensor measures distance using the `getDistanceCM()` function.
- The IR8 sensor array reads line position with `readLineBlack()` returning a value from 0 to 7000.
- A PID controller (using Kp, Ki, Kd) calculates an error based on the line position to adjust motor speeds.
- When an object is detected within the set `maxDistance`, the robot stops by stopping the motors.
- Otherwise, it adjusts motor speeds to follow the line smoothly.

---

## Example Configuration

```cpp
#define MAX_SPEED 160       // Maximum motor speed
float Kp = 0.03;            // Proportional gain
float Ki = 0.0001;          // Integral gain
float Kd = 0.5;             // Derivative gain
float maxDistance = 20;     // Distance in cm to stop before obstacle
```

Modify these parameters in your sketch for tuning performance.

## License

This project is released under the MIT License.

## Support
For more information or help with the IR8 sensor library, visit:
[SVK IR8 GitHub Repository](https://github.com/SVKROBOTICS/IR_8)