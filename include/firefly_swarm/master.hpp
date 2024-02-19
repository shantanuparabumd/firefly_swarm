/******************************************************************************
 * MIT License
 *
 * Copyright (c) 2024 Shantanu Parab
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

/**
 * @file master.hpp
 * @author Shantanu Parab (sparab@umd.edu)
 * @brief Library for master.cpp
 * @version 0.1
 * @date 2022-11-02
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef INCLUDE_FIREFLY_SWARM_MASTER_HPP_
#define INCLUDE_FIREFLY_SWARM_MASTER_HPP_

#include <memory> // Included for shared_ptr
#include <vector> // Included for vector

#include <nav_msgs/msg/odometry.hpp> // Included for Odometry message
#include "firefly_swarm/robot.hpp" // Included for Robot class

using TWIST = geometry_msgs::msg::Twist; // Alias for Twist message
using ODOM = nav_msgs::msg::Odometry; // Alias for Odometry message



/**
 * @class Master
 * @brief A ROS2 pubslisher subscriber node class
 * 
 * Master is a class derived from rclcpp::Node designed to control the entire
 * swarm of robots. It has a timer to call the callback function at a fixed
 * interval. It manages the states of all the robots and runs the firefly
 * algorithm to control the swarm.
 */
class Master : public rclcpp::Node {

 public:
    /**
    * @brief Construct a new Master:: Master object
    * 
    * The constructor initializes the master node with the robot array and the number of nodes
    * It creates a timer to call the process_callback function every 1000ms
    * The process_callback function checks if all robots have reached the goal 
    * and then calls the firefly_algorithm function.
    * @param robot_array 
    * @param nodes 
    */
    Master(std::vector<std::shared_ptr<Robot>> const &robot_array, int nodes);

    /**
    * @brief  The main function to run the firefly algorithm
    * 
    * The function checks if all robots have reached the goal and then
    * calls the firefly_algorithm function.
    * It keeps the track of the generations and the robots that have reached the goal
    * If all the robots have reached the goal, it calls the firefly_algorithm function
    */
    void process_callback();


    /**
    * @brief Function to run the firefly algorithm
    * 
    * The function runs the firefly algorithm for the robots
    * It iterates through the robot array and checks if the robot has reached the goal
    * If the robot has reached the goal, it calls the firefly_inner function
    * @param j 
    */
    void firefly_algorithm(int j);

    /**
    * @brief The firefly_inner function is used to implement the firefly algorithm
    * The algorithms checks if the robot has reached the goal, if not, it checks the following conditions
    * If the robot has detected an object, it moves towards the object
    * If the intensity of the current robot is less than the robot, it moves towards the robot
    * If the intensity of the current robot is greater than the robot, it moves randomly to 
    * explore the environment
    * @param j 
    */
    void firefly_inner(int j);
    
    /**
    * @brief Check if all robots have reached the goal
    * 
    * The function checks if all robots have reached the goal
    * It iterates through the robot array and checks if the robot has reached the goal
    * @return true 
    * @return false 
    */
    bool check_all_robots_reached_goal();
    
    /**
    * @brief Test function to move the robots in a circle
    * 
    * Given the number of nodes and radius, the function moves the robot each
    * robot to a point on the circle which is equally spaced.
    *
    * @param radius 
    */
    void circle(double radius);

private:
  rclcpp::TimerBase::SharedPtr timer_;  ///< Timer to trigger publishing.

  rclcpp::Publisher<TWIST>::SharedPtr  publisher_;  ///< The publisher object.

  std::vector<std::shared_ptr<Robot>> robot_array; ///< Array of robot objects.

  int nodes; ///< Number of robots in the swarm.
  int generations = 0; ///< Number of generations.
  double radius = 5.0; ///< Radius of the circle.

  int robot = 0; ///< Index of the robot.


};

#endif  // INCLUDE_FIREFLY_SWARM_MASTER_HPP_
