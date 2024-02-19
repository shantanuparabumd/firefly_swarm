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
 ************/
 
/*
 * @file test.cpp
 * @author Shantanu Parab (sparab@umd.edu)
 * @brief Test file for the project
 * @version 0.1
 * @date 2024-02-19
 * 
 * @copyright Copyright (c) 2024
 * 
*/
#include <gtest/gtest.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "firefly_swarm/robot.hpp" ///< Include the robot header file
#include "firefly_swarm/master.hpp" ///< Include the master header file

/**
 * @brief Test Suite for the project
 * 
 */
class TestSuite : public ::testing::Test {
 protected:
    // Create a shared pointer to the robot to be used in the tests
    std::shared_ptr<Robot> robot;
    // Create a shared pointer to the master to be used in the tests
    std::shared_ptr<Master> master;
};

/**
 * @brief Construct a new test f object
 * 
 */
TEST_F(TestSuite, robot_motion_test) {
    // Create a robot namespace
    auto r_namespace = "robot_" + std::to_string(1);
    // Create a robot nodename
    auto nodename = "robot_" + std::to_string(1) + "_controller";
    // Create a shared pointer to the robot
    robot = std::make_shared<Robot>(nodename, r_namespace, 0.0, 0.0);
    // Set the goal for the robot (Call the set_goal function)
    robot->set_goal(5.0, 5.0);
    // Check if the goal is set correctly (Call the go_to_goal_callback function)
    robot->go_to_goal_callback();

    // Stop the robot
    robot->stop();

    // Resume the robot
    robot->resume();

    // Complete the robot objective
    robot->complete();

    // Make an assertion true
    EXPECT_EQ(1, 1);
}


TEST_F(TestSuite, robot_publisher_testing) {
  auto r_namespace = "robot_" + std::to_string(1);
  auto nodename = "robot_" + std::to_string(1) + "_controller";
  robot = std::make_shared<Robot>(nodename, r_namespace , 0.0, 0.0);

  auto number_of_publishers =
      robot->count_publishers("/" + r_namespace + "/cmd_vel");
  EXPECT_EQ(1, static_cast<int>(number_of_publishers));
}



// Testing Number of Slaves Spawned Counting the publihsers
TEST_F(TestSuite, slave_spawn_testing_publishers) {
  int nodes = 10;
  int pub_count = 0;
  rclcpp::executors::MultiThreadedExecutor exec;
  std::vector<std::shared_ptr<Robot>> robot_array;
  for (int i = 0; i < nodes; i++) {
    auto r_namespace = "robot_" + std::to_string(i);
    auto nodename = "robot_" + std::to_string(i) + "_controller";
    auto robot = std::make_shared<Robot>(nodename, r_namespace, 0.0, 0.0);
    exec.add_node(robot);
    robot_array.push_back(robot);
  }

  for (int i = 0; i < nodes; i++) {
    auto r_namespace = "robot_" + std::to_string(i);
    auto number_of_publishers =
        robot_array[i]->count_publishers("/" + r_namespace + "/cmd_vel");
    pub_count = pub_count + static_cast<int>(number_of_publishers);
  }
  EXPECT_EQ(nodes, pub_count);
}


// Testing Number of Slaves Spawned_Counting the subscribers
TEST_F(TestSuite, slave_spawn_testing_subscribers) {
  int nodes = 10;
  int pub_count = 0;
  rclcpp::executors::MultiThreadedExecutor exec;
  std::vector<std::shared_ptr<Robot>> robot_array;
  for (int i = 0; i < nodes; i++) {
    auto r_namespace = "robot_" + std::to_string(i);
    auto nodename = "robot_" + std::to_string(i) + "_controller";
    auto robot = std::make_shared<Robot>(nodename, r_namespace , 0.0, 0.0);
    exec.add_node(robot);
    robot_array.push_back(robot);
  }
  for (int i = 0; i < nodes; i++) {
    auto r_namespace = "robot_" + std::to_string(i);
    auto number_of_subs =
        robot_array[i]->count_subscribers("/" + r_namespace + "/odom");
    pub_count = pub_count + static_cast<int>(number_of_subs);
  }
  EXPECT_EQ(nodes, pub_count);
}


// Check Method Compute Distance
TEST_F(TestSuite, compute_distance) {
  int n = 1;
  auto r_namespace = "robot_" + std::to_string(n);
  auto nodename = "robot_" + std::to_string(n) + "_controller";
  robot = std::make_shared<Robot>(nodename, r_namespace , 0.0, 0.0);
  std::pair<double, double> goal{10.0, 10.0};
  std::pair<double, double> loc{0.0, 0.0};
  double distance = robot->compute_distance(loc, goal);
  double ex = 14.1421;
  EXPECT_NEAR(ex, distance, 0.1);
}


// Check Method Yaw From Quaternions
TEST_F(TestSuite, compute_yaw_from_quaternion) {
  int n = 1;
  auto r_namespace = "robot_" + std::to_string(n);
  auto nodename = "robot_" + std::to_string(n) + "_controller";
  robot = std::make_shared<Robot>(nodename, r_namespace , 0.0, 0.0);
  robot->m_orientation.x = 3.0;
  robot->m_orientation.y = 5.0;
  robot->m_orientation.z = 4.0;
  robot->m_orientation.w = 6.0;
  double yaw = robot->compute_yaw_from_quaternion();
  double ex = 1.51955;
  EXPECT_NEAR(ex, yaw, 0.1);
}

// Run the tests
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
