# Drone Simulation with Position and Attitude Control

This repository contains a C++ codebase for simulating a drone using ROS (Robot Operating System) and Gazebo. The code provides position control and attitude control functionalities, making it suitable for implementing Model Predictive Control (MPC) using MAVROS (MAVLink ROS package).
## Prerequisites

Before running the drone simulation code, ensure that you have the following prerequisites installed on your system:

- ROS (Robot Operating System): Make sure you have ROS installed and properly set up. You can refer to the official ROS documentation for installation instructions specific to your operating system.
- Gazebo: Install Gazebo, which is a popular robotics simulator. You can find installation instructions on the Gazebo website.
- Mavros: Mavros package must be installed.

## Installation

To install and use this drone simulation code, follow these steps:

1. Clone the repository to your local machine using the following command:

   ```bash
   git clone https://github.com/ender-yagcilar/DroneSim.git
   ```

2. Change your current directory to the cloned repository:

   ```bash
   cd drone_pkg
   ```

3. Build the ROS workspace using `catkin_make`:

   ```bash
   catkin_make
   ```

4. Source the ROS workspace:

   ```bash
   source devel/setup.bash
   ```

## Usage

To run the drone simulation with position and attitude control, follow these steps:

1. Launch the PX4 Iris Gazebo simulation environment along with the drone model in PX4-Autopilot folder:

   ```bash
   cd PX4-Autopilot/
   make px4_sitl gazebo
   ```

2. In a separate terminal, run the MAVROS:

   ```bash
   roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.133:14557"
   ```

3. In another terminal, run the attitude control node:

   ```bash
   rosrun drone_pkg drone_node
   ```
