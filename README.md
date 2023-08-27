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
   - `platoon.launch`: A basic lane changing maneuver.
   - `static_obstacle.py`: A static obstacle avoidance scenario.
   - `overtake.py`: A dynamic obstacle avoidance scenario.
   - `braking.py`: Abrupt braking for the ego vehicle within a platoon.
- **config**: Configuration files for setting up simulation parameters and vehicle properties.
- **scripts**: Python scripts implementing the proposed control approach and scenario setups.
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






For questions or inquiries, please contact blourenco217@gmai.com.