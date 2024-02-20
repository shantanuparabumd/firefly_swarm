
# Solution Search using Firefly Algorithm.

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT) 
[![firefly_swarm_build](https://github.com/shantanuparabumd/firefly_swarm/actions/workflows/main.yml/badge.svg)](https://github.com/shantanuparabumd/firefly_swarm/actions/workflows/main.yml)


https://github.com/shantanuparabumd/firefly_swarm/assets/112659509/f2ab9e1d-7388-4148-8345-a570da8585a5


## Authors

|Name|ID|Email|
|:---:|:---:|:---:|
|Shantanu Parab|119208625|sparab@umd.edu|


## Introduction

 In the modern era of robotics and artificial intelligence, the concept of swarm intelligence has gained significant attention due to its potential to solve complex problems through the collaboration of multiple simple agents. One such algorithm, the Firefly Algorithm, draws inspiration from the collective behavior of fireflies in nature and has been applied to various optimization and search problems.

This project focuses on the implementation of the Firefly Algorithm for searching suboptimal solutions, particularly objects of red color, within a simulated environment using a swarm of TurtleBots equipped with cameras and LiDAR sensors. The simulation environment is created using Gazebo, a powerful tool for simulating robotic systems in complex and realistic environments.

The primary objective of the project is to explore the capabilities of the Firefly Algorithm in guiding the swarm of TurtleBots to efficiently locate and identify objects of interest based on their color characteristics. By leveraging the sensor data collected by the TurtleBots, including camera images and LiDAR scans, the algorithm orchestrates the collective movement of the swarm to effectively search the environment and locate red-colored objects.

Through this project, we aim to demonstrate the effectiveness and applicability of swarm-based approaches, such as the Firefly Algorithm, in addressing real-world search and optimization tasks. Additionally, the integration of simulation tools like Gazebo provides a platform for testing and validating algorithms in a controlled and customizable environment before deployment in real-world scenarios.

## Dependencies

- OS: Ubuntu Linux 20.04
- ROS Version: ROS2 Galactic
- C++



## SystemSetup

In Ubuntu Linux 20.04 LTS version Install ```ROS2 Galactic```. The Installation documentation can be found [here](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html).

After successfully setting up the ROS2 Galactic, create a workspace.
Source the ROS2,

```sh
source /opt/ros/galactic/setup.bash
```

Create the directory

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Clone the repository,

```sh
git clone https://github.com/shantanuparabumd/firefly_swarm.git
```

Now, check the dependencies,

```sh
# cd if you're still in the ``src`` directory with the ``firefly_swarm`` clone
cd ..
rosdep install -i --from-path src --rosdistro galactic -y
```

and build the package,

```sh
colcon build --packages-select firefly_swarm
```

## Run the module

Once the package is build successfully, to run the module.

```sh
ros2 launch firefly_swarm gazebo_launch.py node_count:=20
```

The ```node_count``` parameter is number of robots user wants to launch. Default value is 10.
In another terminal run after all robots are spawned, to start the firefly search algorithm run the below command

```sh
ros2 run firefly_swarm master 20
```

here again, ```20``` is the number of robots the user wants to spawn.

## Unit Testing

```sh
source install/setup.bash
colcon test --packages-select firefly_swarm
cat log/latest_test/firefly_swarm/stdout_stderr.log
```

to get the test coverage

```sh
rm -rf build/firefly_swarm/
colcon build --cmake-args -DCOVERAGE=1 --packages-select firefly_swarm
cat log/latest_build/firefly_swarm/stdout_stderr.log
```

to generate the coverage report

```sh
source install/setup.bash
ros2 run firefly_swarm generate_coverage_report.bash
```

## Cpplint and Cppcheck

To check the Cpplint and Cppcheck results, check ```results/``` directory. To run the check,

```sh
sh cpplint.sh
sh cppcheck.sh
```

## Known bugs

- While launching 20+ robots at once, ensure that your system has updated GPU drivers and they are proprietary and tested.
- If the Gazebo Simulation fails, restart the launch file. Check whether the previous  ```gzserver``` is not running in the background, using ```ps -a``` in the terminal.
- In some conditions, the robots are stuck within each other or in the obstacle, in such cases the iteration won't terminate.
- Manually move the robot that is stuck using the Gazebo move tool.




