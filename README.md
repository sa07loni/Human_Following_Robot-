Human-Following Robot 🤖🚶‍♂
This project is a human-following robot designed to track and follow a person in real time using a combination of sensors and an Arduino-based control system.

🔧 Components Used
Arduino Uno – The brain of the robot, processing sensor inputs and controlling motors.

Ultrasonic Sensor (HC-SR04) – Detects the distance from the target human to maintain safe following distance.

IR Sensors (x2) – Helps in detecting movement direction and avoiding obstacles.

Servo Motor (SG90) – Rotates the ultrasonic sensor for scanning the surroundings.

Motor Driver Module (L298N) – Controls the DC motors for movement.

DC Geared Motors with Wheels (x4) – Provides mobility to the robot.

Chassis – Acrylic base for mounting all components.

Battery Pack – Powers the entire system.

⚙ Working Principle
Detection: The ultrasonic sensor, mounted on a servo motor, scans the surroundings to detect the human target.

Direction Tracking: IR sensors assist in identifying movement direction and nearby obstacles.

Movement Control: Based on sensor inputs, the Arduino Uno sends signals to the motor driver to move forward, stop, or turn.

Safe Distance Maintenance: The robot maintains a pre-defined distance from the target to avoid collisions.

🚀 Features
Real-time human tracking

Obstacle detection and avoidance

Adjustable following distance

Smooth and stable movement with four-wheel drive

💡 Applications
Personal assistant robots

Warehouse or industrial goods following

Elderly assistance

Automated shopping carts
