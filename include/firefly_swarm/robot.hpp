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
 * @file robot.hpp
 * @author Shantanu Parab (sparab@umd.edu)
 * @brief Library for robot.cpp
 * @version 0.1
 * @date 2022-12-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef INCLUDE_FIREFLY_SWARM_ROBOT_HPP_
#define INCLUDE_FIREFLY_SWARM_ROBOT_HPP_

#include <geometry_msgs/msg/quaternion.hpp> // Included for Quaternion message
#include <geometry_msgs/msg/twist.hpp> // Included for Twist message
#include <nav_msgs/msg/odometry.hpp> // Included for Odometry message
#include <rclcpp/rclcpp.hpp> // Include the ROS2 C++ library 
#include <std_msgs/msg/bool.hpp> // Included for Bool message
#include <string> // Included for string
#include <utility> // Included for pair
#include <sensor_msgs/msg/image.hpp> // Included for Image message
#include <sensor_msgs/msg/laser_scan.hpp> // Included for LaserScan message
#include "cv_bridge/cv_bridge.h" // Included for cv_bridge (OPENCV TO ROS)
#include <opencv2/highgui.hpp> // Included for highgui  (OPENCV)



/**
 * @class Robot
 * @brief Robot class to control individual robots
 * 
 * Robot is a class derived from rclcpp::Node designed to  control the individual robots. 
 * It has callback functions for the robot pose, camera and LiDAR data. It also has a timer to call the 
 * callback function at a fixed interval to publish processed images.
 * It publsihes the velocity commands to "cmd_vel" topic and the processed image to "processed_image" topic.
 * 
 */
class Robot : public rclcpp::Node {
 public:
  Robot(std::string node_name, std::string robot_name,double mloc_x,double mloc_y ,bool go_to_goal = false,
        double linear_speed = 0.4, double angular_speed = 0.5)
      : Node(node_name),
        m_robot_name{robot_name},
        m_location{std::make_pair(mloc_x,mloc_y)},
        m_go_to_goal{go_to_goal},
        m_linear_speed{linear_speed},
        m_angular_speed{angular_speed},
        m_roll{0},
        m_pitch{0},
        m_yaw{0},
        m_kv{0.2},
        m_kh{0.5},
        m_goal_x{0.0},
        m_goal_y{0.0}
        {
    
    m_cbg = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    auto command_topic_name = "/" + m_robot_name + "/cmd_vel";
    auto pose_topic_name = "/" + m_robot_name + "/odom";
    auto camera_topic_name = "/" + m_robot_name + "/camera_sensor/image_raw";
    auto scan_topic_name = "/" + m_robot_name + "/scan";
    auto image_topic_name = "/" + m_robot_name + "/processed_image";
    auto goal_flag_topic_name = "/" + m_robot_name + "/goal_reached";

    RCLCPP_INFO_STREAM(this->get_logger(), "Robot Constructor");

    m_publisher_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>(
        command_topic_name, 10);

    m_goal_reached_publisher =
        this->create_publisher<std_msgs::msg::Bool>(goal_flag_topic_name, 10);

    m_image_publisher =
        this->create_publisher<sensor_msgs::msg::Image>(image_topic_name, 10);


    m_subscriber_robot3_pose =
        this->create_subscription<nav_msgs::msg::Odometry>(
            pose_topic_name, 10,
            std::bind(&Robot::robot_pose_callback, this,
                      std::placeholders::_1));


    


    // Set QoS profile
    rclcpp::QoS qos_profile(10);
    qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
    qos_profile.keep_last(10);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_profile.durability(rclcpp::DurabilityPolicy::SystemDefault);

    m_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_name, qos_profile,
            std::bind(&Robot::robot_scan_callback, this, std::placeholders::_1));

    // Create a subscriber for camera messages
    m_camera_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
                camera_topic_name, qos_profile,
                std::bind(&Robot::robot_camera_callback, this, std::placeholders::_1));

    // Call on_timer function 5 times per second
    m_go_to_goal_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / 1)),
        std::bind(&Robot::go_to_goal_callback, this), m_cbg);

    // Call on_timer function 5 times per second
    m_image_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / 1)),
        std::bind(&Robot::image_pub_callback, this), m_cbg);

  }

  /**
   * @brief Set the goal to reach.
   *
   * @param go_to_goal Flag used to perform a transform listener
   * @param x x-coordinate of the goal position.
   * @param y y-coordinate of the goal position.
   */
  void set_goal(double x, double y) {
    m_go_to_goal = true;
    m_goal_x = x;
    m_goal_y = y;
    // RCLCPP_INFO_STREAM(this->get_logger(), "Going to goal: [" << m_goal_x << ","
    //                                                           << m_goal_y
    //                                                           << "]");
  }
  /**
   * @brief Stop the robot from moving
   *
   */
  void stop();

  void obstacle_avoid();

  void resume();
  /**
   * @brief Compute the distance between two points.
   *
   * @param a The first point.
   * @param b The second point.
   * @return double   The distance between a and b.
   */
  double compute_distance(const std::pair<double, double> &a,
                          const std::pair<double, double> &b);

  /**
   * @brief Callback function for the robot3 pose.
   *
   * @param msg Odometry message.
   */
  void robot_pose_callback(const nav_msgs::msg::Odometry &msg);

    /**
    * @brief Read the camera data from the robot and process it
    * 
    * @param msg 
    */
  void robot_camera_callback(const sensor_msgs::msg::Image &msg);

  /**
   * @brief Read scan data from the robot LiDAR
   * 
   * @param msg 
   */
  void robot_scan_callback(const sensor_msgs::msg::LaserScan &msg);
    

  /**
   * @brief Normalizes the angle to be 0 to 2*M_PI
   *
   * @param angle Angle to normalize (rad)
   * @return double Normalized angle (rad)
   */
  double normalize_angle_positive(double angle);

  /**
   * @brief Normalizes the angle to be -M_PI circle to +M_PI circle
   *
   * @param angle Angle to normalize (rad)
   * @return double Normalized angle (rad)
   */
  double normalize_angle(double angle);

  /**
   * @brief Compute the yaw angle from quaternion pose
   *
   * @return double
   */
  double compute_yaw_from_quaternion();

  /**
   * @brief To move to robot by publish velocities on cmd_vel
   *
   * @param linear linear velocity component
   * @param angular angular velocity component
   */
  void move(double linear, double angular);

  /**
   * @brief process to move the robot to a goal
   *
   */
  void go_to_goal_callback();

  /**
   * @brief Compute the intensity/brightness of the firefly based on the distance form human
   * 
   * @return double 
   */
  double get_intensity();

  double compute_angle(double x, double y);

  double compute_depth(int range, double angle);

  geometry_msgs::msg::Quaternion m_orientation;

  std::pair<double, double> m_location;

  void image_pub_callback();

  bool if_reached_goal() { return !m_go_to_goal; }

  std::string get_robot_name() { return m_robot_name; }

  bool check_obstacle(int range,int center,double distance);

  bool object_detected = false;

  std::pair<double, double> objective_location;

  void complete();

  bool reached_object = false;

  bool reroute = false;

  void set_intensity(double intensity){
    m_intensity = intensity;
  };

 private:
  // attributes
  std::string m_robot_name;  //< robot name used for creating namespace
  bool m_go_to_goal;         //< flag to store if the robot has reached position
  
  double m_linear_speed;     //< base linear velocity of robot
  double m_angular_speed;    //< base angular velocity of robot
  double m_roll;             //< rad
  double m_pitch;            //< rad
  double m_yaw;              //< rad
  double m_kv;               //< gain for linear velocity
  double m_kh;               //< gain for angular velocity
  double m_goal_x;           //< x-coordinate of the goal position    
  double m_goal_y;           //< y-coordinate of the goal position
  double m_distance_to_goal; //< distance to goal
  double m_intensity;        //< intensity of the firefly
  std::vector<float> m_ranges_; //< range data from LiDAR
  sensor_msgs::msg::Image m_processed_image_msg; //< processed image message

  
  int reroute_count = 0; //< reroute count


  rclcpp::CallbackGroup::SharedPtr m_cbg; 

  rclcpp::TimerBase::SharedPtr m_timer;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher_cmd_vel;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_goal_reached_publisher;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_subscriber_robot3_pose;
  
  rclcpp::TimerBase::SharedPtr m_go_to_goal_timer;

  rclcpp::TimerBase::SharedPtr m_image_timer;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_camera_subscriber_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_scan_subscriber_;
  
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_publisher;


};
#endif  // INCLUDE_FIREFLY_SWARM_ROBOT_HPP_
