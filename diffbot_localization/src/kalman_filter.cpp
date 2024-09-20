#include "kalman_filter.hpp"

KalmanFilter::KalmanFilter(const std::string& name)
  : rclcpp::Node(name)
  , mean(0.0)
  , inputMotion(0.0)
  , variance(1000.0)
  , measurmentError(0.5)
  , imuReading(0.0)
  , processNoise(4.0)
  , isFirstIteration(true)
  , lastAngularZ(0.0)
{
  odom_sub = create_subscription<nav_msgs::msg::Odometry>(
      "/diffbot/odom_noisy", 10, std::bind(&KalmanFilter::odometryCallback, this, std::placeholders::_1));

  imu_sub = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/out", 10, std::bind(&KalmanFilter::imuCallback, this, std::placeholders::_1));

  kalman_pub = create_publisher<nav_msgs::msg::Odometry>("/diffbot_localization/odom_kalman", 10);
}

void KalmanFilter::odometryCallback(const nav_msgs::msg::Odometry& odom_msg)
{
  if (isFirstIteration)
  {
    mean = odom_msg.twist.twist.angular.z;
    lastAngularZ = odom_msg.twist.twist.angular.z;
    isFirstIteration = false;
    return;
  }

  nav_msgs::msg::Odometry kalman_odom;

  kalman_odom = odom_msg;

  inputMotion = odom_msg.twist.twist.angular.z - lastAngularZ;

  statePrediction();
  measurmentUpdate();

  lastAngularZ = odom_msg.twist.twist.angular.z;

  kalman_odom.twist.twist.angular.z = mean;

  kalman_pub->publish(kalman_odom);
}

void KalmanFilter::imuCallback(const sensor_msgs::msg::Imu& imu_msg)
{
  imuReading = imu_msg.angular_velocity.z;
}

void KalmanFilter::statePrediction()
{
  mean = mean + inputMotion;
  variance = variance + processNoise;
}

void KalmanFilter::measurmentUpdate()
{
  double gain = variance / (variance + measurmentError);
  variance = variance * (1 - gain);
  mean = mean + gain * (imuReading - mean);
}

int main(int argc, char const* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KalmanFilter>("kalman_filter");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
