#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

class KalmanFilter : public rclcpp::Node
{
public:
  KalmanFilter(const std::string& name);

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  void odometryCallback(const nav_msgs::msg::Odometry& odom_msg);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  void imuCallback(const sensor_msgs::msg::Imu& imu_msg);

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr kalman_pub;

  void statePrediction();
  void measurmentUpdate();

  double mean;
  double inputMotion;
  double variance;
  double measurmentError;
  double imuReading;
  double processNoise;
  bool isFirstIteration;
  double lastAngularZ;
};
