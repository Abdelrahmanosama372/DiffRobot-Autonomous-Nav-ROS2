#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

enum class State {
  Free,
  Warning,
  Danger,
};

class SafetyStop : public rclcpp::Node {
public:
  SafetyStop() : rclcpp::Node("safety_stop"){
    declare_parameter<double>("warning_distance", 0.6);
    declare_parameter<double>("danger_distance", 0.2);
    declare_parameter<std::string>("scan_topic", "scan");
    declare_parameter<std::string>("safety_stop_topic", "safety_stop");

    warning_distance = get_parameter("warning_distance").as_double();
    danger_distance = get_parameter("danger_distance").as_double();
    std::string scan_topic = get_parameter("scan_topic").as_string();
    std::string safety_stop_topic = get_parameter("safety_stop_topic").as_string();

    laser_sub = create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, 10, std::bind(&SafetyStop::laserCallback, this, std::placeholders::_1));
    safety_stop_pub = create_publisher<std_msgs::msg::Bool>(safety_stop_topic, 10);

    is_first_msg = true;
    curr_state = State::Free;
    prev_state = State::Free;
  }

private:
  double warning_distance, danger_distance;
  bool is_first_msg;
  State curr_state, prev_state;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_stop_pub;

  void laserCallback(sensor_msgs::msg::LaserScan msg)
  {
    curr_state = State::Free;
    for(auto range : msg.ranges)
    {
      if(range <= warning_distance)
      {
        curr_state = State::Warning;

        if(range <= danger_distance)
        {
          curr_state = State::Danger;
          break;
        }
      }

      if (curr_state != prev_state)
      {
        std_msgs::msg::Bool is_safety_stop;
        if(curr_state == State::Danger)
        {
          is_safety_stop.data = true;
        }
        else if(curr_state == State::Warning)
        {
          is_safety_stop.data = false;
        }
        else if(curr_state == State::Free)
        {
          is_safety_stop.data = false;
        }
        safety_stop_pub->publish(is_safety_stop);
      }


      prev_state = curr_state;

    }
  }
  
};


int main (int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SafetyStop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
