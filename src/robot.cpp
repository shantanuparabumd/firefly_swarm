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
#include "firefly_swarm/robot.hpp" ///< Include the robot header file



//=====================================
void Robot::robot_pose_callback(const nav_msgs::msg::Odometry &msg) {
  m_location.first = msg.pose.pose.position.x;
  m_location.second = msg.pose.pose.position.y;
  m_orientation = msg.pose.pose.orientation;
}

//=====================================
double Robot::compute_distance(const std::pair<double, double> &a,
                               const std::pair<double, double> &b) {
  return sqrt(pow(b.first - a.first, 2) + pow(b.second - a.second, 2));
}

//=====================================
double Robot::compute_yaw_from_quaternion() {

  // Create a quaternion from the orientation
  tf2::Quaternion q(m_orientation.x, m_orientation.y, m_orientation.z,
                    m_orientation.w);

  // Create a matrix from the quaternion
  tf2::Matrix3x3 m(q);

  double roll, pitch, yaw;

  // Get the RPY values from the matrix
  m.getRPY(roll, pitch, yaw);

  // Normalize the angle to be 0 to 2*M_PI
  if (yaw < 0.0) {
    yaw += 2.0 * M_PI;
  }
  return yaw;
}

//=====================================
void Robot::move(double linear, double angular) {
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear;
  msg.angular.z = angular;
  m_publisher_cmd_vel->publish(msg);
}

//=====================================
void Robot::stop() {

  // Make the go_to_goal flag false
  m_go_to_goal = false;

  // Send a zero velocity command
  geometry_msgs::msg::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = 0;
  cmd_vel_msg.angular.z = 0;
  m_publisher_cmd_vel->publish(cmd_vel_msg);

  // Publish the goal reached message
  std_msgs::msg::Bool goal_reached_msg;
  goal_reached_msg.data = true;
  m_goal_reached_publisher->publish(goal_reached_msg);

}

//=====================================
void Robot::go_to_goal_callback() {

  // If the robot has reached the goal, stop the robot i.e return from the function
  if (!m_go_to_goal) return;

  // Store the goal location in a pair
  std::pair<double, double> goal{m_goal_x, m_goal_y};

  // Compute the distance to the goal
  double distance_to_goal = compute_distance(m_location, goal);

  // Check if the robot has rerouted more than 10 times
  if (reroute_count > 10){
    // Set the reroute_count to 0 and stop the robot
    reroute_count = 0;
    // Make the go_to_goal flag false
    m_go_to_goal = false;
    stop();
  }

  // Check if the robot is not within 0.75m of the goal
  if (distance_to_goal > 0.75) {

    // Compute the distance and angle to the goal
    distance_to_goal = compute_distance(m_location, goal);
    double angle_to_goal = std::atan2(m_goal_y - m_location.second, m_goal_x - m_location.first);

    // Compute the difference between the current orientation and the angle to the goal
    double w = angle_to_goal - compute_yaw_from_quaternion();

    // Normalize the angle to be between -PI and PI
    if (w>M_PI){
      w-=2*M_PI;
    }
    else if(w< -M_PI){
      w += 2*M_PI;
    }


    double linear_x = 0;

    // Check if the angle to the goal is less than 0.3535 radians
    // The robot will start linear motion only if it is facing the goal.
    if (abs(w) < 0.3535){
      // proportional control for linear velocity
      linear_x = std::min(m_kv * distance_to_goal, m_linear_speed);
    }
    

    // proportional control for angular velocity
    double angular_z = m_kh * w;

    // Limit the angular velocity to the maximum angular speed
    if (angular_z > 0)
      angular_z = std::min(angular_z, m_angular_speed);
    else
      angular_z = std::max(angular_z, -m_angular_speed);

    // Move the robot
    move(linear_x, angular_z);

  } 
  else {

    // If the robot is within 0.75m of the goal, stop the robot
    stop();
  }
}

//=====================================
void Robot::image_pub_callback() {
  m_image_publisher->publish(m_processed_image_msg);
}

//=====================================
double Robot::get_intensity(){
  return std::min(m_intensity,1.0);
}

//=====================================
void Robot::robot_camera_callback(const sensor_msgs::msg::Image &msg) {

  // Convert the ROS image message to an OpenCV image
  cv_bridge::CvImagePtr cv_ptr;

  // Convert the ROS image message to an OpenCV image
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "cv_bridge exception: " << e.what());
    return;
  }

  // Check if the robot has reached the goal
  if (reached_object){return;}

  // Create a variable for the HSV image
  cv::Mat hsv_image;

  // Convert the BGR image to the HSV image
  cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

  // Create a variable for the red mask
  cv::Mat red_mask;

  // Create a mask for the red color
  cv::inRange(hsv_image, cv::Scalar(0, 120, 99), cv::Scalar(0, 255, 255), red_mask);

  // Create a variable vector for the contours
  std::vector<std::vector<cv::Point>> contours;

  // Find contours of the red color box
  cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // Variables to store the largest contour and its area
  double max_area = 0;
  std::vector<cv::Point> largest_contour;

  // Find the largest contour. Iterate through the contours
  for (const auto &contour : contours) {
    // Get the area of the contour
    double area = cv::contourArea(contour);
    // If the area is greater than the max area, update the max area and the largest contour
    if (area > max_area) {
      max_area = area;
      largest_contour = contour;
    }
  }

  // Get the bounding rectangle of the largest contour
  cv::Rect bounding_rect = cv::boundingRect(largest_contour);

  // Draw the bounding rectangle around the largest contour
  cv::rectangle(cv_ptr->image, bounding_rect, cv::Scalar(0, 255, 0), 2);

  // Get the size of the rectangle
  int rect_area =  bounding_rect.width * bounding_rect.height;

  // Find the centroid of the largest contour
  cv::Moments moments = cv::moments(largest_contour);
  cv::Point centroid(moments.m10 / moments.m00, moments.m01 / moments.m00);

  double angle = 0;
  double depth = 0;

  // Display the area of the rectangle on the image
  cv::putText(cv_ptr->image, "Rect Area : "+ std::to_string(rect_area), cv::Point(10, 180), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
  
  // Check if the largest contour is not empty and the area of the rectangle is greater than 5000
  if (!largest_contour.empty() && rect_area > 5000){

    // Display the object detected message on the image
    cv::putText(cv_ptr->image, "Object Detected", cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);

    // Compute the angle and depth of the centroid
    angle = compute_angle(centroid.x);
    depth = compute_depth(1, angle);

    // Check if the depth is between 0.1 and 3.5
    if (depth < 3.5 && depth > 0.1){

      // Stop the robot so that it can face the object
      stop();

      // Compensate for the current orientation of the robot
      double compensated_angle = angle*M_PI/180 + compute_yaw_from_quaternion();

      // Calculate the x and y coordinates of the object
      double object_x = m_location.first + depth * cos(compensated_angle);
      double object_y = m_location.second + depth * sin(compensated_angle);
     
      // Set the objective location
      objective_location = std::make_pair(object_x, object_y);

      // Calculate the intensity of the robot based on the distance from the object
      m_intensity = std::min(1.0 * exp(-1.0 * pow(depth, 2)),1.0);

      // Set the object detected flag to true
      object_detected = true;
      
    }
    
  } 

  // Display centroid and values on the image
  cv::putText(cv_ptr->image, "Centroid: (" + std::to_string(centroid.x) + ", " + std::to_string(centroid.y) + ")", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
  
  // Display the intensity, angle and depth of the object on the image
  cv::putText(cv_ptr->image, "Intensity: " + std::to_string(m_intensity), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
  cv::putText(cv_ptr->image, "Angle: " + std::to_string(angle), cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
  cv::putText(cv_ptr->image, "Depth: " + std::to_string(depth), cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);

  // Convert the OpenCV image to a ROS image message
  sensor_msgs::msg::Image::SharedPtr processed_image_msg_ptr = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_ptr->image).toImageMsg();

  // Store the pointer to the processed image message to be publsihed later
  m_processed_image_msg = *processed_image_msg_ptr;

}

//=====================================
double Robot::compute_angle(double x){
    // Camera field of view (FOV) in degrees
    double fov_degrees = 80.0;

    // Image dimensions (assuming square image)
    int image_width = 800;

    // Calculate the angle per pixel
    double angle_per_pixel = fov_degrees / image_width;

    // Calculate the distance from the center to the point along the x-axis
    int center_x = image_width / 2;
    int distance_x = x - center_x;

    // Calculate the angle from the center to the point
    double angle_from_center = angle_per_pixel * distance_x;

    return angle_from_center;
}

//=====================================
double Robot::compute_depth(int range,double angle){

  // Convert the angle to integer
  int center = int(angle);

  // Calculate the low and high range of angles to consider
  int low = center - range;
  int high = center + range;

  // Variables to store the sum and count of the distances
  double sum = 0;
  double count = 0;

  // Iterate through the range of angles and compute the sum and count
  for (int i = low; i <= high; ++i) {
    if (i >= 0 && i < static_cast<int>(m_ranges_.size())) {
        sum += m_ranges_[i];
        count += 1; 
    }
    if (i < 0) {
        sum = m_ranges_[static_cast<int>(m_ranges_.size()) + i];
        count += 1;
      
    }
  }

  // Return the average distance
  return sum/count;
}


//=====================================
bool Robot::check_obstacle(int range,int center,double distance){

  // Flag to store if the obstacle is detected
  bool obstacle_detected = false;

  // Calculate the low and high range of angles to consider
  int low = center - range;
  int high = center + range;

  // Iterate through the range of angles and check for obstacles
  for (int i = low; i <= high; ++i) {
    // For positive angles
    
    if (i >= 0 && i < static_cast<int>(m_ranges_.size()) ) {
      // Check if the distance is less than the threshold
      if (m_ranges_[i] < distance) {
        // If the distance is less than the threshold, 
        // set the obstacle_detected flag to true and break
        obstacle_detected = true;
        break;
      }
    }
    // For negative angles convert the negative index to positive index
    if (i < 0) {
      // Check if the distance is less than the threshold
      if (m_ranges_[static_cast<int>(m_ranges_.size()) + i] < distance) {
        // If the distance is less than the threshold, 
        // set the obstacle_detected flag to true and break
        obstacle_detected = true;
        break;
      }
    }
  }
  return obstacle_detected;
}

//=====================================
void Robot::robot_scan_callback(const sensor_msgs::msg::LaserScan &msg)
{
  // Get the ranges from the LaserScan message
  m_ranges_ = msg.ranges;
  
  // Check if the robot has not reacged the goal and if the obstacle is detected
  if (!reached_object && check_obstacle(60,0,0.3)) {
    // Set the reroute flag to true. So that the robot can avoid the obstacle
    reroute = true;
  }

  // If the reroute flag is true, call the obstacle_avoid method
  if (reroute){
    obstacle_avoid();
  }

}

//=====================================
void Robot::obstacle_avoid() {

  // Make the go_to_goal flag false. Stop the robot from moving towards the goal
  m_go_to_goal = false;
  
  // Create a Twist message to publish the velocity command
  geometry_msgs::msg::Twist cmd_vel_msg;

  // Check if there is obstacle within 0.3m in the front of the robot upto 60 degrees FOV
  if(check_obstacle(60,0,0.3)){
    // If there is an obstacle, turn the robot to the right
    cmd_vel_msg.linear.x = 0;
    cmd_vel_msg.angular.z = 0.5;
    m_publisher_cmd_vel->publish(cmd_vel_msg);
  }
  // If there is no obstacle in the front of the robot, move the robot away from the obstacle
  else{
    // Compute the angle to the goal
    double angle_to_goal = std::atan2(m_goal_y - m_location.second, m_goal_x - m_location.first);
    
    // Check if there is an obstacle within 0.5m in the direction of the goal
    if(check_obstacle(90,int(angle_to_goal*180/M_PI),0.5)){
      // If there is an obstacle, move the robot straight till the path is cleared
      cmd_vel_msg.linear.x = 0.3;
      cmd_vel_msg.angular.z = 0.0;
      m_publisher_cmd_vel->publish(cmd_vel_msg);
    }
    else{
      // If the path to goal is clear, resume the robot to move towards the goal.
      resume();
    }
  }
}

//=====================================
void Robot::resume() {
  // Set the go_to_goal flag to true
  m_go_to_goal = true;
  // Set the reroute flag to false
  reroute = false;
  // Increment the reroute count to keep track of the number of reroutes
  reroute_count+=1;

}

//=====================================
void Robot::complete() {
  // Create a Twist message to publish the velocity command
  geometry_msgs::msg::Twist msg;
  // Set the go_to_goal flag to false
  m_go_to_goal = false;
  // Set intensity to 1.0
  m_intensity = 1.0;

  // Publish the velocity command to rotate the robot
  msg.linear.x = 0.0;
  msg.angular.z = 2.0;
  m_publisher_cmd_vel->publish(msg);
  
}

//=====================================
void Robot::set_intensity(double intensity){
        m_intensity = intensity;
    }

//=====================================
bool Robot::if_reached_goal() { return !m_go_to_goal; }

//=====================================
std::string Robot::get_robot_name() { return m_robot_name; }

//=====================================
void Robot::set_goal(double x, double y) {
        m_go_to_goal = true;
        m_goal_x = x;
        m_goal_y = y;     
    }

//=====================================
Robot::Robot(std::string node_name, std::string robot_name,double mloc_x,double mloc_y)
      : Node(node_name),
        m_robot_name{robot_name},  /// Initialize the robot name
        m_location{std::make_pair(mloc_x,mloc_y)}, /// Initialize the robot location
        m_go_to_goal{false}, /// Initialize the go_to_goal flag
        m_linear_speed{0.4}, /// Initialize the linear speed
        m_angular_speed{0.5}, /// Initialize the angular speed
        m_roll{0}, // Initialize the roll
        m_pitch{0}, // Initialize the pitch
        m_yaw{0}, // Initialize the yaw
        m_kv{0.2}, // Initialize the kv linear gain
        m_kh{0.5},  // Initialize the kh angular gain
        m_goal_x{0.0}, // Initialize the goal x
        m_goal_y{0.0}   // Initialize the goal y
        {
    
    m_cbg = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    auto command_topic_name = "/" + m_robot_name + "/cmd_vel";  ///< Create the command topic name
    auto pose_topic_name = "/" + m_robot_name + "/odom"; ///< Create the pose topic name
    auto camera_topic_name = "/" + m_robot_name + "/camera_sensor/image_raw"; ///< Create the camera topic name
    auto scan_topic_name = "/" + m_robot_name + "/scan"; ///< Create the scan topic name
    auto image_topic_name = "/" + m_robot_name + "/processed_image"; ///< Create the image topic name
    auto goal_flag_topic_name = "/" + m_robot_name + "/goal_reached"; ///< Create the goal flag topic name

    RCLCPP_INFO_STREAM(this->get_logger(), "Robot Constructor"); ///< Print the robot constructor message

    // Create a publisher for the velocity command
    m_publisher_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>(
        command_topic_name, 10);

    // Create a publisher for the goal reached message
    m_goal_reached_publisher =
        this->create_publisher<std_msgs::msg::Bool>(goal_flag_topic_name, 10);

    // Create a publisher for the processed image
    m_image_publisher =
        this->create_publisher<sensor_msgs::msg::Image>(image_topic_name, 10);

    // Create a subscriber for the robot pose
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

    // Create a subscriber for the laser scan messages
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