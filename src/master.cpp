/******************************************************************************
 * MIT License
 *
 * Copyright (c) 2024  Shantanu Parab
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
 * @file master.cpp
 * @author Shantanu Parab (sparab@umd.edu)
 * @brief Algorithm to run the robot
 * @version 0.1
 * @date 2022-12-05
 *
 * @copyright Copyright (c) 2022
 *
 */


#include "firefly_swarm/master.hpp"

#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>




using std::placeholders::_1;
using namespace std::chrono_literals;

using TWIST = geometry_msgs::msg::Twist;
using ODOM = nav_msgs::msg::Odometry;
using RCL_NODE_PTR = std::shared_ptr<rclcpp::Node>;


//=====================================
Master::Master(std::vector<std::shared_ptr<Robot>> const &robot_array,
               int nodes)
    : Node("master_node") {
  // Initialize the robot array and the number of nodes
  this->robot_array = robot_array;
  this->nodes = nodes;

  // Variable to the bind the process_callback function
  auto processCallback = std::bind(&Master::process_callback, this);

  // Create a timer to call the process_callback function every 1000ms
  this->timer_ = this->create_wall_timer(1000ms, processCallback);
}



//=====================================
void Master::process_callback() {

  // Check if the generations are less than 100
  if (this->generations < 100) {
      // Check if all robots have reached the goal
      if (this->check_all_robots_reached_goal()) {
        // Log the information
        RCLCPP_INFO_STREAM(this->get_logger(), "All robots reached goal");
        // Check if the robot is less than the number of nodes
        if (robot < this->nodes) {
          // Call the firefly_algorithm function
          this->firefly_algorithm(robot);
          robot++;
        } else {
          // Reset the robot to 0
          robot = 0;
          // Increment the generations
          this->generations++;
          // Log the information
          RCLCPP_INFO_STREAM(this->get_logger(), "Update generation: " << this->generations);
        }
      }
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "Algorithm reached max generations");
  }
}




//=====================================
bool Master::check_all_robots_reached_goal() {
  bool flag = true;
  // Iterate through the robot array
  for (const auto& robot : robot_array) {
    if (!robot->if_reached_goal()) {
      flag = false;
      break;
    }
  }
  return flag;
}



//=====================================
void Master::circle(double radius) {

  // Calculate the angle between the points
  double h = 2 * 3.142 / this->nodes;

  // Initialize the id to 0
  int id = 0;
  // Iterate through the number of nodes
  for (double i = 0.0; i < h * this->nodes; i += h) {
    // Calculate the x and y coordinates from radius and angle
    double a = radius * cos(i);
    double b = radius * sin(i);
    if (i < 2 * 3.14) {
      // Set the goal for the robot
      this->robot_array[id]->set_goal(a, b);
      // Increment the id
      id += 1;
    }
  }
}


//=====================================
void Master::firefly_algorithm(int j) {
    // Iterate through the robot array
    if (j< this->nodes) {
      // Check if the all the robots have reached the goal
      if (check_all_robots_reached_goal()) {
        // Check if the robot has reached the goal
        if (robot_array[j]->reached_object) {
          // Stop the robot and complete the process
          robot_array[j]->stop();
          robot_array[j]->complete();
          robot_array[j]->reroute  = false;
          RCLCPP_INFO_STREAM(this->get_logger(), robot_array[j]->get_robot_name()
            <<" reached Object at " << robot_array[j]->objective_location.first <<","
              <<robot_array[j]->objective_location.second);

          // Call the firefly_inner function
          firefly_inner(j);
          j++;
        } else {
          RCLCPP_INFO_STREAM(this->get_logger(), robot_array[j]->get_robot_name()
            <<" has intensity "<< robot_array[j]->get_intensity());
          // Call the firefly_inner function
          firefly_inner(j);
          j++;
        }
      }
    }

}


//=====================================
void Master::firefly_inner(int j) {
  // Log the information of the current robot
  RCLCPP_INFO_STREAM(this->get_logger(), "#####"<< robot_array[j]->get_robot_name()
    << "Intensity: "<< robot_array[j]->get_intensity()<< "############");
  // Iterate through the all other robots in the  array
  for (int i = 0 ; i< this->nodes ; i++) {

      // Get robot i location
      std::pair<double, double> robot_i_location = robot_array[i]->get_pose();
      
          // Check if any robot has reached the goal
          if (i!= j && !robot_array[i]->reached_object) {
            // If the robot has not reached the goal, check the following conditions

            // Check if the robot has detected an object
            if (robot_array[i]->object_detected) {
              // Get the location of the object the robot has detected
              std::pair<double, double> objective_location = robot_array[i]->objective_location;

              

              // Calculate the direction towards the object location
              double direction_x = objective_location.first - robot_i_location.first;
              double direction_y = objective_location.second - robot_i_location.second;
              double magnitude = sqrt(direction_x * direction_x + direction_y * direction_y);

              // Check if the distance to the object is less than 0.5
              if (magnitude < 0.5) {
                // If the distance is less than 0.5, stop the robot and complete the process
                RCLCPP_INFO_STREAM(this->get_logger(), robot_array[i]->get_robot_name()
                  << ":Object at" << objective_location.first << "," << objective_location.second);
                this->robot_array[i]->stop();
                robot_array[i]->complete();
                robot_array[i]->reached_object = true;
              } else {  ///< If the distance is greater than 0.5, move the robot towards the object
                // Define the range for the  step
                double step_size = 2.0;

                // Normalize the direction vector
                direction_x /= magnitude;
                direction_y /= magnitude;

                // Calculate the new goal position by adding the direction vector with the step size
                double x = robot_i_location.first + direction_x * step_size;
                double y = robot_i_location.second + direction_y * step_size;

                // Set the new goal position
                this->robot_array[i]->set_goal(x, y);

                // Log the information
                RCLCPP_INFO_STREAM(this->get_logger(), robot_array[i]->get_robot_name()
                  <<" moving towards object " << x << "," << y << ":Object at"
                    << objective_location.first << "," << objective_location.second);
              }
              // If the robot did not detect an object
          // Check if the intensity of the current robot is greater than the  robot
            } else if (this->robot_array[i]->get_intensity()< this->robot_array[j]->get_intensity()) {

            // Get robot j location
            std::pair<double, double> robot_j_location = robot_array[j]->get_pose();

            // Set the attractiveness of the robot based on the intensity
            double beta = 1.0 * robot_array[j]->get_intensity();
            // Set the gamma value (absorption coefficient)
            double gamma = 0.02;
            // Calculate the distance between the robots
            double distance = robot_array[i]->compute_distance(robot_i_location, robot_j_location);

            // Calculate the new goal position based on the distance and attractiveness
            double x = robot_i_location.first +
              beta*exp(-gamma*pow(distance, 2))*(robot_j_location.first - robot_i_location.first);
            double y = robot_i_location.second +
              beta*exp(-gamma*pow(distance, 2))*(robot_j_location.second - robot_i_location.second);

            // Set the new goal position
            robot_array[i]->set_goal(x, y);

            // Calculate the attraction between the robots in terms of distance it is moving.
            double attract = robot_array[i]->compute_distance(robot_i_location, std::make_pair(x, y));


            // Log the information
            RCLCPP_INFO_STREAM(this->get_logger(), robot_array[i]->get_robot_name()<<
            "("<< robot_array[i]->get_intensity() << ") moving towards " << robot_array[j]->get_robot_name() <<
            " ("<< robot_array[j]->get_intensity()<< ") Attraction: "<< attract);

            RCLCPP_INFO_STREAM(this->get_logger(), "Distance: "<< distance<< " Location: "
            << robot_i_location.first << ","<< robot_i_location.second
            << " New Goal: "<< x<< ","<< y);

          // If no object is detected and the intensity of the current robot is less than the robot
          // Then move the robot randomly to explore the environment
          } else {
            // Define the range for the random step
            double step_size = 1.0;

            // Generate random offsets for x and y coordinates
            double random_x_offset = (2.0 * static_cast<double>(rand()) / RAND_MAX - 1.0) * step_size;
            double random_y_offset = (2.0 * static_cast<double>(rand()) / RAND_MAX - 1.0) * step_size;

            // Calculate the new goal position by adding random offsets
            double x = robot_i_location.first + random_x_offset;
            double y = robot_i_location.second + random_y_offset;

            // Set the new goal position
            this->robot_array[i]->set_goal(x, y);
            RCLCPP_INFO_STREAM(this->get_logger(), robot_array[i]->get_robot_name()
                <<" moving randomly to "<< x <<","<< y);
          }
          } else {
            RCLCPP_INFO_STREAM(this->get_logger(), robot_array[i]->get_robot_name()<< "("
              << robot_array[i]->get_intensity()<< ") is not moving");
          }
        }
}
