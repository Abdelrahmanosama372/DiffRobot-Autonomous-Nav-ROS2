#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <random>

class NoisyController : public rclcpp::Node
{
public:
    NoisyController(const std::string &name);

private:

    void joint_callback(const sensor_msgs::msg::JointState &msg);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;

    nav_msgs::msg::Odometry odom_msg;

    double wheel_radius;
    double wheel_separation;

    double left_wheel_prev_pos;
    double right_wheel_prev_pos;

    rclcpp::Time prev_time;

    double x_;
    double y_;
    double theta_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster;
    geometry_msgs::msg::TransformStamped transform_stamped_msg;
};

#endif // SIMPLE_CONTROLLER_HPP
