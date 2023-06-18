# Autonomous Jet-Racer

## Introduction
This project aims to control the Jet-Racer vehicle autonomously, allowing it to navigate along a wall and complete a full lap as quickly as possible. The vehicle is equipped with a lidar sensor for detecting obstacles and maintaining a specific distance from the wall.

## Lidar Data Collection Method
To simplify the handling of a list containing over 1000 distance values from the lidar sensor, it was divided into 36 sections. This division allows for easy reference to individual or multiple sections of the circle without the need to determine which values correspond to a specific area.

## Vehicle Movement Algorithm Concept
The vehicle's control system utilizes two PI (Proportional-Integral) controllers to regulate the distance from the wall and maintain parallelism to it. These controllers enable the vehicle to move along the wall smoothly.

### Function of the First PI Controller:
This controller adjusts the vehicle's steering to ensure that two green lines (distances at angles of 20 degrees and -20 degrees from the perpendicular line) have the same value.

### Function of the Second PI Controller:
This controller adjusts the vehicle's speed to maintain a constant distance from the wall (in this case, 65 cm).

Additionally, with the use of specific conditions, the vehicle can turn (1). In cases where a turn is not feasible, the vehicle reverses with a steering angle (2), thereby increasing its maneuverability.

When a wall is detected ahead, the algorithm compares the lengths of the lidar lines (represented by green lines) to determine which side has more free space and steers in that direction.
If the algorithm detects a critical distance from the wall, the vehicle reverses and steers in the opposite direction to smoothly navigate around challenging obstacles.

## Autonomous Jet-Racer Drive
Authors: Marek T, Michał D, Mikołaj L
Lap Time: 28.50 seconds

## Prerequisites
- Jet-Racer vehicle with lidar sensor
- Python environment with required dependencies

## How to Run
1. Connect to the Jet-Racer vehicle using NoMachine and launch the Ubuntu virtual machine.
2. In the NoMachine terminal, execute the following commands:
   - `roscore` - maintains a constant connection to the virtual machine.
   - `roslaunch jetracer jetracer.launch` - launches the node for controlling the vehicle.
   - `roslaunch jetracer lidar.launch` - starts the lidar sensor.
3. In the Ubuntu terminal, execute the following command:
   - `rosrun jetracer face.detect.launch` - runs the script with our program.

## Conclusion
The vehicle successfully completes the entire track, moving at maximum speed. The lap time achieved is 28.50 seconds, which is the fastest time among all the groups. 
Please refer to the code files for more details and customization options.

