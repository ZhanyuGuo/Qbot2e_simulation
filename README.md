# Qbot2e Simulation

## Quick Start

1. Download the repo.
    ```bash
    git clone https://github.com/ZhanyuGuo/Qbot2e_simulation.git
    ```

2. Compile.
    ```bash
    cd <YOUR_PATH>/Qbot2e_simulation/
    catkin_make  # or catkin build
    ```

3. Config the env.
    ```bash
    # each new terminal
    cd <YOUR_PATH>/Qbot2e_simulation/
    source devel/setup.bash
    export TURTLEBOT3_MODEL=burger  # options: burger, waffle, waffle_pi

    # or add the above two lines to ~/.bashrc
    nano ~/.bashrc
    # add: source <YOUR_PATH>/Qbot2e_simulation/devel/setup.bash
    # add: export TURTLEBOT3_MODEL=burger
    
    source ~/.bashrc  # or open a new terminal
     ```

4. Run robot in a empty gazebo world.
   ```bash
   # NOTE: you have to source and export again if you have not modified the ~/.bashrc
   roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

   # in a new terminal, use the keyboard to control it
   roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
   ```

    ![demo1](./asset/demo1.png)

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

    Note that `/cmd_vel` is the control input and `/odom` is the localization, they are the same topics we encounter in MATLAB, i.e., `/mobile_base/commands/velocity` and `/odom`.

2. Use MATLAB-ROS to control it!
   1. Requirements: `ROS Toolbox`.

   2. Launch the robot.
        ```bash
        roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
        ```

   3. Run `src/turtlebot3_controller/src/matlab/demo.m`.

    ![demo2](./asset/demo2.gif)

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

    ![demo3](./asset/demo3.gif)
