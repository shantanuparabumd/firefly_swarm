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


#include <string>  // Included for string
#include <utility>  // Included for pair
#include <vector>  // Included for vector
#include <geometry_msgs/msg/quaternion.hpp>  // Included for Quaternion message
#include <geometry_msgs/msg/twist.hpp>  // Included for Twist message
#include <nav_msgs/msg/odometry.hpp>  // Included for Odometry message
#include <rclcpp/rclcpp.hpp>  // Include the ROS2 C++ library
#include <std_msgs/msg/bool.hpp>  // Included for Bool message
#include <sensor_msgs/msg/image.hpp>  // Included for Image message
#include <sensor_msgs/msg/laser_scan.hpp>   // Included for LaserScan message
#include "cv_bridge/cv_bridge.h"  // Included for cv_bridge (OPENCV TO ROS)
#include <opencv2/highgui.hpp>  // Included for highgui  (OPENCV)
#include <geometry_msgs/msg/transform_stamped.hpp>  // Included for TransformStamped message
#include "tf2/LinearMath/Matrix3x3.h"  // Included for Matrix3x3
#include "tf2/LinearMath/Quaternion.h"  // Included for Quaternion
#include "tf2/exceptions.h"  // Included for tf2 exceptions
#include "tf2_ros/buffer.h"  // Included for buffer
#include "tf2_ros/transform_listener.h"  //  Included for transform listener


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
    /**
     * @brief Construct a new Robot object
     * 
     * The constructor initializes the robot node with the robot name and the goal location.
     * It creates a timer to call the go_to_goal_callback function every 100ms
     * It also creates a timer to call the image_pub_callback function every 100ms
     * The go_to_goal_callback function checks if the robot has reached the goal and then moves the robot towards the goal
     * The image_pub_callback function publishes the processed image to the "processed_image" topic
     * 
     * @param node_name 
     * @param robot_name 
     * @param mloc_x 
     * @param mloc_y 
     */
    Robot(std::string robot_name, std::string node_name, double mloc_x, double mloc_y);

    /**
    * @brief Set the goal to reach.
    *
    * @param go_to_goal Flag used to perform a transform listener
    * @param x x-coordinate of the goal position.
    * @param y y-coordinate of the goal position.
    */
    void set_goal(double x, double y);

   /**
    * @brief Stop command
    * 
    * Stops the robot by sending a zero velocity command
    * Also makes the go_to_goal flag false
    */
    void stop();

   /**
    * @brief Method to avoid the obstacle
    * 
    * The robot avoids the obstacle by turning away from the obstacle and then moving away from it.
    * The robot keeps on turning until it finds a clear path to move towards the goal.
    */
    void obstacle_avoid();

   /**
    * @brief Method to resume the robot to move towards the goal
    * 
    * The robot resumes to move towards the goal by setting the go_to_goal flag to true
    */
    void resume();


  /**
    * @brief Computes the Euclidian Distance
    * 
    * Given a pair of points, computes the distance between them
    * @param a (x,y) coordinate of point a
    * @param b (x,y) coordinate of point b
    * @return double distance between the points
    */
    double compute_distance(const std::pair<double, double> &a,
                          const std::pair<double, double> &b);

   /**
    * @brief Pose Callback method
    * 
    * Stores the current location and orientation of the robot
    * in the global posiion variable of the robot
    * @param msg Odometry message from the robot "/odom" topic
    */
    void robot_pose_callback(const nav_msgs::msg::Odometry &msg);

   /**
    * @brief Read the camera data from the robot and process it
    * 
    * The robot reads the camera data and processes it to detect the red color box
    * It computes the angle and depth of the centroid of the box and the intensity of the box
    * It also computes the x and y coordinates of the box and stores it in the objective_location variable
    * 
    * @param msg 
    */
    void robot_camera_callback(const sensor_msgs::msg::Image &msg);

  /**
    * @brief Callback method for the LiDAR scan.
    * 
    * The robot checks for obstacles in the LiDAR scan and if an obstacle is detected,
    * takes the necessary action to avoid the obstacle.
    *
    * @param msg 
    */
    void robot_scan_callback(const sensor_msgs::msg::LaserScan &msg);

  /**
    * @brief Evaluates the Yaw from Quaternion
    * 
    * Converts the quaternion to RPY and returns the yaw
    * The yaw is converted to the range [0, 2*PI]
    *
    * @return double Yaw in radians
    */
    double compute_yaw_from_quaternion();

  /**
   * @brief To move to robot by publish velocities on "cmd_vel"
   *
   * @param linear linear velocity component
   * @param angular angular velocity component
   */
    void move(double linear, double angular);

  /**
    * @brief Send robot to the alloted location
    * 
    * While the robot is not at the goal, it computes the distance and angle to the goal
    * and moves the robot towards the goal using proportional control
    * If the robot is within 0.75m of the goal, it stops the robot
    */
    void go_to_goal_callback();

  /**
 * @brief Getter for intensity
 * 
 * Returns the intensity of the robot limited to 1.0
 * @return double 
 */
    double get_intensity();

   /**
    * @brief Compute the angle between the x axis of robot and the centroid of the object
    * 
    * Using the camera field of view and the image dimensions, the angle of the object 
    * from the center of the image is computed.
    *
    * @param x x coordinate of the centroid of the object
    * @return double Normalized angle (rad)
    */
    double compute_angle(double x);


   /**
    * @brief Compute the distance to the object
    * 
    * Using the angle to the centroid of object and the LiDAR data,
    * the depth of the object is computed.
    *
    * @param range Range of angles to consider
    * @param angle Angle to normalize (degrees)
    * @return double Distance (meters)
    */
    double compute_depth(int range, double angle);

    /**
    * @brief Publishes the image message
    * 
    * Publishes the processed image message to the topic "processed_image".
    * This is used to display the processed image in the GUI
    */
    void image_pub_callback();

   /**
    * @brief The robot checks for obstacles in the LiDAR scan in a particular direction
    * 
    * @param range 360 degree range scan from the LiDAR
    * @param center Center point for the direction in the LiDAR scan
    * @param distance Threshold distance to check for obstacle
    * 
    * @return bool  
    */

    bool check_obstacle(int range, int center, double distance);

   /**
    * @brief Method to indiacte that the robot has found the solution
    * 
    * The robot completes the task by stopping the robot and setting the go_to_goal flag to false
    * the robot also starts rotating to indicate that it has found the solution visually.
    */
    void complete();

    /**
     * @brief Set the intensity object
     * 
     * @param intensity 
     */
    void set_intensity(double intensity);

    /**
     * @brief Set the go to goal object
     * 
     * @param go_to_goal 
     */
    bool if_reached_goal();

    /**
     * @brief Get the robot name object
     * 
     * @return std::string 
     */
    std::string get_robot_name();

    /**
     * @brief Get the location robot
     * 
     * @return std::pair<double, double> 
     */
    std::pair<double, double> get_pose();

    /**
     * @brief Set the location robot
     * 
     * @param x 
     * @param y 
     */
    void set_location(double x, double y);



    // Flags

    bool object_detected = false;  ///< Flag to indicate if the object is detected

    bool reached_object = false;  ///< Flag to indicate if the robot has reached the object

    bool reroute = false;  ///< Flag to indicate if the robot has to reroute



    geometry_msgs::msg::Quaternion m_orientation;  ///< Orientation of the robot

    

    std::pair<double, double> objective_location;  ///< Location of the object

 private:
  // attributes
  std::string m_robot_name;  //< robot name used for creating namespace

  std::pair<double, double> m_location;  ///< Location of the robot

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
  double m_distance_to_goal;  //< distance to goal
  double m_intensity;        //< intensity of the firefly
  std::vector<float> m_ranges_;  //< range data from LiDAR
  sensor_msgs::msg::Image m_processed_image_msg;  //< processed image message


  int reroute_count = 0;  //< reroute count


  rclcpp::CallbackGroup::SharedPtr m_cbg;

  rclcpp::TimerBase::SharedPtr m_timer;  ///< Timer to trigger publishing.

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher_cmd_vel;  ///< The publisher object for velocity.

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    m_goal_reached_publisher;   ///< The publisher object for goal status.

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
     m_subscriber_robot3_pose;  ///< The subscriber object for robot pose.

  rclcpp::TimerBase::SharedPtr m_go_to_goal_timer;  ///< Timer to trigger go to goal callback.

  rclcpp::TimerBase::SharedPtr m_image_timer;  ///< Timer to trigger image publishing callback.

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
     m_camera_subscriber_;  ///< The subscriber object for camera data.

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
     m_scan_subscriber_;  ///< The subscriber object for LiDAR data.

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
     m_image_publisher;  ///< The publisher object for processed image.
};
#endif  // INCLUDE_FIREFLY_SWARM_ROBOT_HPP_
