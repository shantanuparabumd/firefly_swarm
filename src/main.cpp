/******************************************************************************
 * MIT License
 *
 * Copyright (c) 2022 Jay Prajapati, Shail Shah and Shantanu Parab
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
 * @file main.cpp
 * @author Shantanu Parab (sparab@umd.edu)
 * @brief Algorithm to run the robot
 * @version 0.1
 * @date 2022-12-05
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "firefly_swarm/master.hpp"
#include "firefly_swarm/robot.hpp"

/**
 * @brief main function entry point
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  int nodes = 10;
  if (argc > 0) {
    std::string arg = argv[1];
    try {
      std::size_t pos;
      nodes = std::stoi(arg, &pos);
      if (pos < arg.size()) {
        std::cerr << "Trailing characters after number: " << arg << '\n';
      }
    } catch (std::invalid_argument const &ex) {
      std::cerr << "Invalid number: " << arg << '\n';
    } catch (std::out_of_range const &ex) {
      std::cerr << "Number out of range: " << arg << '\n';
    }
  }
  std::vector<std::shared_ptr<Robot>> robot_array;
  for (int i = 0; i < nodes; i++) {
    auto r_namespace = "firefly_" + std::to_string(i);
    auto nodename = "firefly_" + std::to_string(i) + "_controller";
    auto robot = std::make_shared<Robot>(nodename, r_namespace, i-nodes/2, 0.0);
    exec.add_node(robot);
    robot_array.push_back(robot);
  }
  auto node = std::make_shared<Master>(robot_array, static_cast<int>(nodes));
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
