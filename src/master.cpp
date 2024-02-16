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
 * @file master.cpp
 * @author Shantanu Parab (sparab@umd.edu)
 * @brief Algorithm to run the robot
 * @version 0.1
 * @date 2022-12-05
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <string>

#include "firefly_swarm/master.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

using TWIST = geometry_msgs::msg::Twist;
using ODOM = nav_msgs::msg::Odometry;
using RCL_NODE_PTR = std::shared_ptr<rclcpp::Node>;

/**
 * @brief Construct a new Master:: Master object
 * 
 * @param robot_array 
 * @param nodes 
 */
Master::Master(std::vector<std::shared_ptr<Robot>> const &robot_array,
               int nodes)
    : Node("master_node") {
  this->robot_array = robot_array;
  this->nodes = nodes;
  auto processCallback = std::bind(&Master::process_callback, this);
  this->timer_ = this->create_wall_timer(1000ms, processCallback);
  // this->circle(radius);
  
}


// void Master::process_callback() {
//   // RCLCPP_INFO_STREAM(this->get_logger(), "iterating Publisher array");
//   if (this->generations < 100){
//       if (this->check_all_robots_reached_goal()) {
//         RCLCPP_INFO_STREAM(this->get_logger(), "All robots reached goal");
//         this->circle(radius);
//         this->generations++;
//         radius = radius + 1.0;
//         if (radius > 10.0){
//           radius = 5.0;
//         }
//       }
//   // RCLCPP_INFO_STREAM(this->get_logger(), "Current generation: " << this->generations);
//   }
//   else{
//     RCLCPP_INFO_STREAM(this->get_logger(), "Algorithm reached max generations");
//   }
  
  
// }

/**
 * @brief Process callback logger info
 * 
 */
void Master::process_callback() {
  // RCLCPP_INFO_STREAM(this->get_logger(), "iterating Publisher array");
  if (this->generations < 100){
    // RCLCPP_INFO_STREAM(this->get_logger(), "Current generation: " << this->generations);
      if (this->check_all_robots_reached_goal()) {
        RCLCPP_INFO_STREAM(this->get_logger(), "All robots reached goal");
        if(robot < this->nodes){
          this->firefly_algorithm(robot);
          robot++;
        }
        else{
          robot = 0;
          this->generations++;
          RCLCPP_INFO_STREAM(this->get_logger(), "Update generation: " << this->generations);
        }

      }
  
  }
  else{
    RCLCPP_INFO_STREAM(this->get_logger(), "Algorithm reached max generations");
  }
  
  
}


bool Master::check_all_robots_reached_goal() {
  bool flag = true;
    for (const auto& robot : robot_array) {
      if (!robot->if_reached_goal()) {
        flag = false;
        // RCLCPP_INFO_STREAM(this->get_logger(), "Robot "<<robot->get_robot_name()<<"reached goal Intensity: "<<robot->get_intensity());
      }
    }
    return flag;
  }


void Master::circle(double radius) {
  double h = 2 * 3.142 / this->nodes;
  int id = 0;
  for (double i = 0.0; i < h * this->nodes; i += h) {
    double a = radius * cos(i);
    double b = radius * sin(i);
    if (i < 2 * 3.14) {
      this->robot_array[id]->set_goal(a, b);
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "robot_id " << id << " a " << a << " b " << b);
      id += 1;
    }
  }
}


void Master::firefly_algorithm(int j) {
    if (j<this->nodes){
      if(check_all_robots_reached_goal()){
        if (robot_array[j]->reached_object){
          robot_array[j]->stop();
          robot_array[j]->complete();
          robot_array[j]->reroute  = false;
          RCLCPP_INFO_STREAM(this->get_logger(),robot_array[j]->get_robot_name()<<" reached Object at " << robot_array[j]->objective_location.first <<","<<robot_array[j]->objective_location.second);
        }
        else{
          RCLCPP_INFO_STREAM(this->get_logger(),robot_array[j]->get_robot_name()<<" has intensity "<< robot_array[j]->get_intensity());
          firefly_inner(j);
          j++;
        }
      }
    }

}



void Master::firefly_inner(int j){
  for (int i = 0;i<this->nodes;i++){
          if (i!=j && !robot_array[i]->reached_object){
            if(robot_array[i]->object_detected){

              std::pair<double,double> objective_location = robot_array[i]->objective_location;
              // Calculate the direction towards the objective location
              double direction_x = objective_location.first - this->robot_array[i]->m_location.first;
              double direction_y = objective_location.second - this->robot_array[i]->m_location.second;
              double magnitude = sqrt(direction_x * direction_x + direction_y * direction_y);

              if (magnitude < 0.75){
                RCLCPP_INFO_STREAM(this->get_logger(),robot_array[i]->get_robot_name()<<" reached Object at " << objective_location.first <<","<<objective_location.second);
                this->robot_array[i]->stop();
                robot_array[i]->complete();
                robot_array[i]->reached_object = true;
              }
              else{
                // Define the range for the random step
                double step_size = 2.0; // Adjust as needed

                // Normalize the direction vector
                direction_x /= magnitude;
                direction_y /= magnitude;

                // Calculate the new goal position by adding the direction vector with the step size
                double x = this->robot_array[i]->m_location.first + direction_x * step_size;
                double y = this->robot_array[i]->m_location.second + direction_y * step_size;

                // Set the new goal position
                this->robot_array[i]->set_goal(x, y);
                
              

                RCLCPP_INFO_STREAM(this->get_logger(),robot_array[i]->get_robot_name()<<" moving towards object " << x <<","<<y);
                // RCLCPP_INFO_STREAM(this->get_logger(),robot_array[i]->get_robot_name()<<" detected Object at " << objective_location.first <<","<<objective_location.second);
              }
            }

          else if (this->robot_array[i]->get_intensity()<this->robot_array[j]->get_intensity()){
            double beta = 2.0;
            double gamma = 0.1;
            double distance = this->robot_array[i]->compute_distance(this->robot_array[i]->m_location,this->robot_array[j]->m_location);
            double x = this->robot_array[i]->m_location.first + beta*exp(-gamma*pow(distance,2))*(this->robot_array[j]->m_location.first - this->robot_array[i]->m_location.first);
            double y = this->robot_array[i]->m_location.second + beta*exp(-gamma*pow(distance,2))*(this->robot_array[j]->m_location.second - this->robot_array[i]->m_location.second);
            this->robot_array[i]->set_goal(x,y);
            RCLCPP_INFO_STREAM(this->get_logger(),robot_array[i]->get_robot_name()<<" moving towards "<<robot_array[j]->get_robot_name());
            // RCLCPP_INFO_STREAM(this->get_logger(),robot_array[i]->get_intensity()<<" moving towards "<<robot_array[j]->get_intensity());
          }
          else{
            // Define the range for the random step
            double step_size = 3.0; // Adjust as needed

            // Generate random offsets for x and y coordinates
            double random_x_offset = (2.0 * (double)rand() / RAND_MAX - 1.0) * step_size;
            double random_y_offset = (2.0 * (double)rand() / RAND_MAX - 1.0) * step_size;

            // Calculate the new goal position by adding random offsets
            double x = this->robot_array[i]->m_location.first + random_x_offset;
            double y = this->robot_array[i]->m_location.second + random_y_offset;

            // Set the new goal position
            this->robot_array[i]->set_goal(x, y);      
            RCLCPP_INFO_STREAM(this->get_logger(),robot_array[i]->get_robot_name()<<" moving randomly to "<<x <<","<<y);
          }
          }
          else{
            RCLCPP_INFO_STREAM(this->get_logger(),robot_array[i]->get_robot_name()<<" reached Object at " << robot_array[i]->objective_location.first <<","<<robot_array[i]->objective_location.second);
          }
        }
}
