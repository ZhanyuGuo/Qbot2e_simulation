# Qbot2e Simulation

## Quick Start

1. Download the repository.
    ```bash
    git clone https://github.com/ZhanyuGuo/Qbot2e_simulation.git
    ```

2. Compile the repository.
    ```bash
    cd <YOUR_PATH>/Qbot2e_simulation/
    catkin_make  # or catkin build
    ```

3. Config the environment.
    ```bash
    # in each new terminal
    cd <YOUR_PATH>/Qbot2e_simulation/
    source devel/setup.bash
    export TURTLEBOT3_MODEL=burger  # options: burger, waffle, waffle_pi

    # or
    source script/env.sh

    # or add the above two lines to ~/.bashrc
    nano ~/.bashrc
    # add: source <YOUR_PATH>/Qbot2e_simulation/devel/setup.bash
    # add: export TURTLEBOT3_MODEL=burger
    # save and exit, and then
    source ~/.bashrc  # or re-open the terminal
    ```

4. Run the robot in an empty gazebo world.
   ```bash
   # NOTE: you have to source and export again if you have NOT modified the ~/.bashrc
   roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

   # in a new terminal, use the keyboard to control it
   roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
   ```

    <p align="center">
        <img src="asset/demo1.png">
    </p>

## Details

1. Use `rostopic list`, we get the followings
    ```bash
    /clock
    /cmd_vel
    /gazebo/link_states
    /gazebo/model_states
    /gazebo/parameter_descriptions
    /gazebo/parameter_updates
    /gazebo/performance_metrics
    /gazebo/set_link_state
    /gazebo/set_model_state
    /imu
    /joint_states
    /odom
    /rosout
    /rosout_agg
    /scan
    /tf
    ```

    Note that `/cmd_vel` is the control input and `/odom` is the localization, they are the same topics we encounter within MATLAB in real world Qbot, i.e., `/mobile_base/commands/velocity` and `/odom`.

2. Use MATLAB-ROS to control it!
   1. Requirements: `ROS Toolbox`.

   2. Launch the robot.
        ```bash
        roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
        ```

   3. Run `src/turtlebot3_controller/src/matlab/demo.m` in MATLAB.

    <p align="center">
        <img src="asset/demo2.gif">
    </p>

3. Use Python-ROS to control it!
   1. Requirements: `numpy`.

   2. Launch the robot.
        ```bash
        roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
        ```
    
    3. Run controller.
        ```bash
        rosrun turtlebot3_controller demo.py
        ```

    <p align="center">
        <img src="asset/demo3.gif">
    </p>
