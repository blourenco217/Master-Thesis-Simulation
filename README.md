# Master Thesis Simulation
ROS Package - Simulation Environment

Repository containing a simulation environment as part of a Dissertation submitted in partial partial fulfilment for the degree of Master of Science in Electrical and Computer Engineering at [Instituto Superior Técnico - Lisbon University](https://tecnico.ulisboa.pt/pt/).

![alt text](results/overtake.gif)

# Abstract

## Keywords

## Overview

The goal of this simulation environment is to evaluate the proposed approach for truck platooning using a Nonlinear Model Predictive Control (MPC) strategy within a Cooperative Adaptive Cruise Control (CACC) framework. The approach's performance is tested in various scenarios, including obstacle avoidance, lane-changing maneuvers, overtaking, and abrupt braking for the ego vehicle within a platoon.

## Contents

- **launch**: Contains launch files for starting the simulation environment.
   - `world.launch`: Launches the simulation highway environment.
   - `platoon.launch`: Launches the simulation environment for basic lane-changing maneuvers within a platoon.
   - `static_obstacle.launch`: Initiates the simulation environment for static obstacle avoidance scenarios.
   - `overtake.launch`: Sets up the simulation environment for overtaking scenarios involving dynamic obstacles.
   - `braking.launch`: Launches the simulation environment to evaluate the ego vehicle's response to abrupt braking scenarios.
   - `rviz.launch`: Launches the RViz visualization tool for real-time visualization of the simulation environment.
- **config**: Configuration files for setting up simulation parameters and vehicle properties.
- **scr**: Python scripts implementing the proposed control approach and scenario setups.
- **worlds**: Gazebo world files representing different road and traffic scenarios.
- **rviz**: RViz configuration files for visualizing the simulation environment.




## Installation

1. Clone this repository into your ROS workspace:

   ```bash
   git clone https://github.com/blourenco217/master-thesis-simulation.git
   ```

2. Build the ROS package:
   ```bash
   cd ~/your-ros-workspace
   catkin_make
   ```

# Usage






For questions or inquiries, please contact blourenco217@gmail.com.