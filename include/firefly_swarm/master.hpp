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
#include <memory>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>


#include "firefly_swarm/robot.hpp"

using TWIST = geometry_msgs::msg::Twist;
using ODOM = nav_msgs::msg::Odometry;

/**
 * @brief Class for the robot master
 *
 */
class Master : public rclcpp::Node {
 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<TWIST>::SharedPtr
      publisher_;  // Change to publish to custom robot array
  std::vector<std::shared_ptr<Robot>> robot_array;
  int nodes;
  int generations = 0;
  double radius = 5.0;

  int robot = 0;

 public:
  Master(std::vector<std::shared_ptr<Robot>> const &robot_array, int nodes);
  /**
   * @brief process callback to kepp the ode running
   *
   */
  void process_callback();
  /**
   * @brief Create co ordinate points for robots to form a circel
   *
   * @param radius Radius of the circle
   */
  void firefly_algorithm(int j);

  void firefly_inner(int j);

  /**
   * @brief Compute the attraction force between two fireflies
   * 
   * @param a 
   * @param b 
   * @return double 
   */
//   double compute_attraction(const std::pair<double, double> &a,
//                           const std::pair<double, double> &b);

    bool check_all_robots_reached_goal();

    void circle(double radius);


};

#endif  // INCLUDE_FIREFLY_SWARM_MASTER_HPP_
