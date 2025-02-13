# 🏁 Track_sim.m
`Track_sim.m` simulates autonomous driving on an elliptical track composed of yellow and blue cones using the ERP42 model.    
It includes ROS communication, perception and decision-making, a Pure Pursuit controller, path generation, and tracking algorithms.
<br> <br>
## Table of Contents
1. [How to Run](#how-to-run)
2. [Main Components](#main-components)
3. [Dependencies](#dependencies)
4. [Simulation Video](#simulation-video)
<br> <br>
## How to Run
To run the simulation, first, ensure that the `erp42_ws` ROS workspace is properly set up and launched:

1. Open a terminal and navigate to the `erp42_ws` directory.
2. Source the workspace:
   ```sh
   source devel/setup.bash
   ```
3. Launch the ERP42 ROS environment:
   ```sh
   roslaunch erp42 erp42.launch
   ```
4. Open MATLAB and navigate to the directory containing `Track_sim.m`.
5. Run the script:
   ```matlab
   Track_sim
   ```
<br> <br>
## Main Components
- **Path Definition**: Loads or generates a reference trajectory.
- **Pure Pursuit Control**: Computes steering angle based on lookahead point.
- **Vehicle Model**: Simulates motion based on kinematic constraints.
- **Visualization**: Displays vehicle trajectory and tracking performance.
<br> <br>
## Dependencies
- **MATLAB R2024b**
- **MATLAB Toolboxes:**
  - Control System Toolbox
  - Robotics System Toolbox
  - Automated Driving Toolbox
- **ROS (Robot Operating System)**
- **RViz**: For monitoring sensor data
- **Gazebo**: For visualizing the simulation
<br> <br>
## Simulation Video
![ERP42 Simulation](images/simulation.gif)

