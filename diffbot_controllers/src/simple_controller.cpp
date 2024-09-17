#include "simple_controller.hpp"

SimpleController::SimpleController(const std::string &name) 
    : rclcpp::Node(name),
      left_wheel_prev_pos(0),
      right_wheel_prev_pos(0),
      x_(0), y_(0), theta_(0) {

    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_separation", 0.17);

    wheel_radius = get_parameter("wheel_radius").as_double();
    wheel_separation = get_parameter("wheel_separation").as_double();

    cmd_vel_pub = create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands", 10);
    vel_sub = create_subscription<geometry_msgs::msg::Twist>("/diffbot/cmd_vel",10, std::bind(&SimpleController::vel_callback, this, std::placeholders::_1));

    joint_sub = create_subscription<sensor_msgs::msg::JointState>("/joint_states",10, std::bind(&SimpleController::joint_callback, this, std::placeholders::_1));
    odometry_pub = create_publisher<nav_msgs::msg::Odometry>("/diffbot/odom", 10);

    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;

    transform_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    transform_stamped_msg.header.frame_id = "odom";
    transform_stamped_msg.child_frame_id = "base_footprint";

    speed_conversion << wheel_radius/2, wheel_radius/2, wheel_radius/wheel_separation, -wheel_radius/wheel_separation;
    RCLCPP_INFO_STREAM(get_logger(), "The conversion matrix is \n" << speed_conversion);

     prev_time = get_clock()->now();
}

void SimpleController::vel_callback(const geometry_msgs::msg::Twist &msg){
    Eigen::Vector2d speed(msg.linear.x, msg.angular.z);
    Eigen::Vector2d wheels_vel = speed_conversion.inverse() * speed;

    std_msgs::msg::Float64MultiArray cmd_vel;
    cmd_vel.data.push_back(wheels_vel.coeff(1));
    cmd_vel.data.push_back(wheels_vel.coeff(0));

    RCLCPP_INFO_STREAM(get_logger(), "publishing data to /simple_velocity_controller/commands ... \n" 
        << wheels_vel.coeff(0) << " " << wheels_vel.coeff(1));

    cmd_vel_pub->publish(cmd_vel);
}

void SimpleController::joint_callback(const sensor_msgs::msg::JointState &msg)
{
    double dp_right = msg.position.at(0) - right_wheel_prev_pos;
    double dp_left = msg.position.at(1) - left_wheel_prev_pos;

    rclcpp::Time msg_time = msg.header.stamp;
    rclcpp::Duration duration = msg_time - prev_time;

    double omega_right = dp_right / duration.seconds();
    double omega_left = dp_left / duration.seconds();

    prev_time = msg_time;
    right_wheel_prev_pos = msg.position.at(0);
    left_wheel_prev_pos = msg.position.at(1);

    double linear = (wheel_radius * omega_right + wheel_radius * omega_left) / 2;
    double angular = (wheel_radius * omega_right - wheel_radius * omega_left) / wheel_separation;

    double d_s = (wheel_radius * dp_right + wheel_radius * dp_left) / 2;
    double d_theta = (wheel_radius * dp_right - wheel_radius * dp_left) / wheel_separation;

    theta_ += d_theta;
    x_ += d_s * cos(theta_);
    y_ += d_s * sin(theta_);

    tf2::Quaternion q;
    q.setRPY(0,0,theta_);

    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    odom_msg.header.stamp = get_clock()->now();
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.twist.twist.linear.x = linear;
    odom_msg.twist.twist.angular.z = angular;

    transform_stamped_msg.header.stamp = get_clock()->now();
    transform_stamped_msg.transform.translation.x = x_;
    transform_stamped_msg.transform.translation.y = y_;
    transform_stamped_msg.transform.rotation.x = q.x();
    transform_stamped_msg.transform.rotation.y = q.y();
    transform_stamped_msg.transform.rotation.z = q.z();
    transform_stamped_msg.transform.rotation.w = q.w();

    odometry_pub->publish(odom_msg);
    transform_broadcaster->sendTransform(transform_stamped_msg);
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SimpleController>("simple_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
