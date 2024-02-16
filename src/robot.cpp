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
 * @file robot.cpp
 * @author Shantanu Parab (sparab@umd.edu)
 * @brief slave robot methods definations
 * @version 0.1
 * @date 2024-11-02
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "firefly_swarm/robot.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

/**
 * @brief Pose Callback method
 * 
 * @param msg 
 */
void Robot::robot_pose_callback(const nav_msgs::msg::Odometry &msg) {
  m_location.first = msg.pose.pose.position.x;
  m_location.second = msg.pose.pose.position.y;
  m_orientation = msg.pose.pose.orientation;
}

/**
 * @brief Normalizing the yaw angle
 * 
 * @param angle 
 * @return double 
 */
double Robot::normalize_angle_positive(double angle) {
  const double result = fmod(angle, 2.0 * M_PI);
  if (result < 0) return result + 2.0 * M_PI;
  return result;
}

/**
 * @brief Normalizing the yaw angle
 * 
 * @param angle 
 * @return double 
 */
double Robot::normalize_angle(double angle) {
  const double result = fmod(angle + M_PI, 2.0 * M_PI);
  if (result <= 0.0) return result + M_PI;
  return result - M_PI;
}






/**
 * @brief Computes the Euclidian Distance
 * 
 * @param a 
 * @param b 
 * @return double 
 */
double Robot::compute_distance(const std::pair<double, double> &a,
                               const std::pair<double, double> &b) {
  return sqrt(pow(b.first - a.first, 2) + pow(b.second - a.second, 2));
}

/**
 * @brief Evaluates the Yaw from Quaternion
 * 
 * @return double 
 */
double Robot::compute_yaw_from_quaternion() {
  tf2::Quaternion q(m_orientation.x, m_orientation.y, m_orientation.z,
                    m_orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  if (yaw < 0.0) {
    yaw += 2.0 * M_PI;
  }
  return yaw;
}

/**
 * @brief Sends command to move to each robots
 * 
 * @param linear 
 * @param angular 
 */
void Robot::move(double linear, double angular) {
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear;
  msg.angular.z = angular;
  m_publisher_cmd_vel->publish(msg);
  // RCLCPP_INFO_STREAM(this->get_logger(), "Moving "<<this->m_robot_name<< msg.linear.x << " " << msg.angular.z);
}

/**
 * @brief Stop command
 * 
 */
void Robot::stop() {
  m_go_to_goal = false;
  geometry_msgs::msg::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = 0;
  cmd_vel_msg.angular.z = 0;
  m_publisher_cmd_vel->publish(cmd_vel_msg);

  std_msgs::msg::Bool goal_reached_msg;
  goal_reached_msg.data = true;
  m_goal_reached_publisher->publish(goal_reached_msg);
}




/**
 * @brief Send robot to the alloted location
 * 
 */
void Robot::go_to_goal_callback() {
  if (!m_go_to_goal) return;

  std::pair<double, double> goal{m_goal_x, m_goal_y};
  double distance_to_goal = compute_distance(m_location, goal);
  if (reroute_count > 10){
    reroute_count = 0;
    m_go_to_goal = false;
    stop();
  }
  if (distance_to_goal > 0.25) {
    distance_to_goal = compute_distance(m_location, goal);
    double angle_to_goal =
        std::atan2(m_goal_y - m_location.second, m_goal_x - m_location.first);

    // RCLCPP_INFO_STREAM(this->get_logger(), "Angle to Goal: " << angle_to_goal<<" Yaw: "<< compute_yaw_from_quaternion());
    
    // angle to rotate to face the goal
    double w = angle_to_goal - compute_yaw_from_quaternion();

    if (w>M_PI){
      w-=2*M_PI;
    }
    else if(w< -M_PI){
      w += 2*M_PI;
    }

    // w = w*-1;

    double linear_x = 0;

    if (abs(w) < 0.3535){
      // proportional control for linear velocity
      linear_x = std::min(m_kv * distance_to_goal, m_linear_speed);
    }
    

    // proportional control for angular velocity
    double angular_z = m_kh * w;
    if (angular_z > 0)
      angular_z = std::min(angular_z, m_angular_speed);
    else
      angular_z = std::max(angular_z, -m_angular_speed);

    move(linear_x, angular_z);
  } else {
    // RCLCPP_INFO_STREAM(this->get_logger(),
    //                    "********** Goal reached **********");
    stop();
  }
}

void Robot::image_pub_callback() {
  m_image_publisher->publish(m_processed_image_msg);
}

double Robot::get_intensity(){
  return m_intensity;
}




/**
 * @brief Read the camera data from the robot and process it
 * 
 * @param msg 
 */
void Robot::robot_camera_callback(const sensor_msgs::msg::Image &msg) {
  // Convert the ROS image message to an OpenCV image
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "cv_bridge exception: " << e.what());
    return;
  }
  if (reached_object){return;}
  // Detect red color box in the image
  cv::Mat hsv_image;
  cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

  cv::Mat red_mask;
  cv::inRange(hsv_image, cv::Scalar(0, 120, 99), cv::Scalar(0, 255, 255), red_mask);

  // Find contours of the red color box
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // Find the largest contour
  double max_area = 0;
  std::vector<cv::Point> largest_contour;
  for (const auto &contour : contours) {
    double area = cv::contourArea(contour);
    if (area > max_area) {
      max_area = area;
      largest_contour = contour;
    }
  }

  // Draw bounding box around the largest contour
  cv::Rect bounding_rect = cv::boundingRect(largest_contour);
  cv::rectangle(cv_ptr->image, bounding_rect, cv::Scalar(0, 255, 0), 2);
  // Draw bounding box around the largest contour
  // Get the size of the rectangle
  int rect_area =  bounding_rect.width * bounding_rect.height;

  // Find the centroid of the largest contour
  cv::Moments moments = cv::moments(largest_contour);
  cv::Point centroid(moments.m10 / moments.m00, moments.m01 / moments.m00);
  double angle = 0;
  double depth = 0;
  cv::putText(cv_ptr->image, "Rect Area : "+ std::to_string(rect_area), cv::Point(10, 180), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
  if (!largest_contour.empty() && rect_area > 5000 && rect_area < 20000){
    cv::putText(cv_ptr->image, "Object Detected", cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);

    // Compute the angle and depth of the centroid
    angle = compute_angle(centroid.x, centroid.y);
    depth = compute_depth(1, angle);

    if (depth < 3.5 && depth > 0.1){
      stop();
      // Compensate for the current orientation of the robot
      double compensated_angle = angle*M_PI/180 + compute_yaw_from_quaternion();

      // Calculate the x and y coordinates of the object
      double object_x = m_location.first + depth * cos(compensated_angle);
      double object_y = m_location.second + depth * sin(compensated_angle);
      // RCLCPP_INFO_STREAM(this->get_logger(), "Robot "<<get_robot_name()<< " at: (" << m_location.first << ", " << m_location.second << ", " << compute_yaw_from_quaternion()*180/M_PI);
      // RCLCPP_INFO_STREAM(this->get_logger(), "Angle, Depth: (" << angle << ", " << depth << ")");
      // RCLCPP_INFO_STREAM(this->get_logger(), "Object detected at: (" << object_x << ", " << object_y << ")");
      objective_location = std::make_pair(object_x, object_y);

      m_intensity = 1.0 * exp(-1.0 * pow(depth, 2));
      object_detected = true;
      
    }
    
  } 

  // Display centroid and values
  cv::putText(cv_ptr->image, "Centroid: (" + std::to_string(centroid.x) + ", " + std::to_string(centroid.y) + ")", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
  cv::putText(cv_ptr->image, "Intensity: " + std::to_string(m_intensity), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
  cv::putText(cv_ptr->image, "Angle: " + std::to_string(angle), cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
  cv::putText(cv_ptr->image, "Depth: " + std::to_string(depth), cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);

  // Convert the OpenCV image to a ROS image message
  sensor_msgs::msg::Image::SharedPtr processed_image_msg_ptr = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_ptr->image).toImageMsg();

  // Now you can access the image message through the pointer
  m_processed_image_msg = *processed_image_msg_ptr;

  

}



double Robot::compute_angle(double x,double y){
    // Camera field of view (FOV) in degrees
    double fov_degrees = 80.0;

    // Image dimensions (assuming square image)
    int image_width = 800;
    int image_height = 800;

    // Calculate the angle per pixel
    double angle_per_pixel = fov_degrees / image_width;

    // Calculate the distance from the center to the point along the x-axis
    int center_x = image_width / 2;
    int distance_x = x - center_x;

    // Calculate the angle from the center to the point
    double angle_from_center = angle_per_pixel * distance_x;

    return angle_from_center;
}

double Robot::compute_depth(int range,double angle){
  int center = int(angle);
  int low = center - range;
  int high = center + range;
  double sum = 0;
  double count = 0;
  for (int i = low; i <= high; ++i) {
    if (i >= 0 && i < m_ranges_.size()) {
        sum += m_ranges_[i];
        count += 1; 
    }
    if (i < 0) {
        sum = m_ranges_[m_ranges_.size() + i];
        count += 1;
      
    }
  }
  return sum/count;
}


bool Robot::check_obstacle(int range,int center,double distance){
  bool stop_robot = false;
  int low = center - range;
  int high = center + range;
  for (int i = low; i <= high; ++i) {

    if (i >= 0 && i < m_ranges_.size()) {
      if (m_ranges_[i] < distance) {
        stop_robot = true;
        break;
      }
    }
    if (i < 0) {
      if (m_ranges_[m_ranges_.size() + i] < distance) {
        stop_robot = true;
        break;
      }
    }
  }
  return stop_robot;
}


void Robot::robot_scan_callback(const sensor_msgs::msg::LaserScan &msg)
{
  // Get the ranges from the LaserScan message
  m_ranges_ = msg.ranges;
  
  // Check if the robot has not reacged the goal and if the obstacle is detected
  if (!reached_object && check_obstacle(60,0,0.3)) {
    reroute = true;
  }
  if (reroute){
    obstacle_avoid();
  }

}

void Robot::obstacle_avoid() {
  m_go_to_goal = false;
  
  geometry_msgs::msg::Twist cmd_vel_msg;
  if(check_obstacle(60,0,0.3)){
    // RCLCPP_INFO_STREAM(this->get_logger(),get_robot_name()<<" Avoiding Obstacle");/
    cmd_vel_msg.linear.x = 0;
    cmd_vel_msg.angular.z = 0.5;
    m_publisher_cmd_vel->publish(cmd_vel_msg);
  }
  else{
    double angle_to_goal =
        std::atan2(m_goal_y - m_location.second, m_goal_x - m_location.first);
    // RCLCPP_INFO_STREAM(this->get_logger(),get_robot_name()<<" Angle to Goal: "<<angle_to_goal*180/M_PI);
    if(check_obstacle(90,int(angle_to_goal*180/M_PI),0.5)){
      cmd_vel_msg.linear.x = 0.3;
      cmd_vel_msg.angular.z = 0.0;
      m_publisher_cmd_vel->publish(cmd_vel_msg);
      // RCLCPP_INFO_STREAM(this->get_logger(),get_robot_name()<<" Moving Away from Obstacle");
    }
    else{
      resume();
    }
  }
}

void Robot::resume() {
  m_go_to_goal = true;
  reroute = false;
  reroute_count+=1;

}


void Robot::complete() {
  geometry_msgs::msg::Twist msg;
  m_go_to_goal = false;
  msg.linear.x = 0.0;
  msg.angular.z = 2.0;
  m_publisher_cmd_vel->publish(msg);
}