#include "noisy_controller.hpp"

NoisyController::NoisyController(const std::string &name) 
    : rclcpp::Node(name),
      left_wheel_prev_pos(0),
      right_wheel_prev_pos(0),
      x_(0), y_(0), theta_(0) {

    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_separation", 0.17);

    wheel_radius = get_parameter("wheel_radius").as_double();
    wheel_separation = get_parameter("wheel_separation").as_double();

    joint_sub = create_subscription<sensor_msgs::msg::JointState>("/joint_states",10, std::bind(&NoisyController::joint_callback, this, std::placeholders::_1));
    odometry_pub = create_publisher<nav_msgs::msg::Odometry>("/diffbot/odom_noisy", 10);

    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint_ekf";
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;

    transform_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    transform_stamped_msg.header.frame_id = "odom";
    transform_stamped_msg.child_frame_id = "base_footprint_ekf";

    prev_time = get_clock()->now();
}

void NoisyController::joint_callback(const sensor_msgs::msg::JointState &msg)
{
   unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine noise_generator(seed);
    std::normal_distribution<double> left_encoder_noise(0.0, 0.005);
    std::normal_distribution<double> right_encoder_noise(0.0, 0.005);

    double wheel_encoder_left = msg.position.at(1) + left_encoder_noise(noise_generator);
    double wheel_encoder_right = msg.position.at(0) + right_encoder_noise(noise_generator);

    double dp_right = wheel_encoder_right - right_wheel_prev_pos;
    double dp_left = wheel_encoder_left - left_wheel_prev_pos;

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
    auto node = std::make_shared<NoisyController>("noisy_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
