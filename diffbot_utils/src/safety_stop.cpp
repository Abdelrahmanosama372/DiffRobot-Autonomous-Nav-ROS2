#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <twist_mux_msgs/action/detail/joy_turbo__struct.hpp>
#include <twist_mux_msgs/action/joy_turbo.hpp>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

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
    zones_pub = create_publisher<visualization_msgs::msg::MarkerArray>("safety_zones", 10);

    increase_speed_client = rclcpp_action::create_client<twist_mux_msgs::action::JoyTurbo>(this, "joy_turbo_increase");
    decrease_speed_client = rclcpp_action::create_client<twist_mux_msgs::action::JoyTurbo>(this, "joy_turbo_decrease");

    is_first_msg = true;
    curr_state = State::Free;
    prev_state = State::Free;

    visualization_msgs::msg::Marker danger_zone;
    danger_zone.id = 0;
    danger_zone.type = visualization_msgs::msg::Marker::CYLINDER;
    danger_zone.action = visualization_msgs::msg::Marker::ADD;
    danger_zone.scale.x = danger_distance * 2;
    danger_zone.scale.y = danger_distance * 2;
    danger_zone.scale.z = 0.001;
    danger_zone.color.r = 1.0;
    danger_zone.color.g = 0.0;
    danger_zone.color.b = 0.0;
    danger_zone.color.a = 0.5;
    zones.markers.push_back(danger_zone);

    visualization_msgs::msg::Marker warning_zone = danger_zone;
    warning_zone.id = 1;
    warning_zone.scale.x = warning_distance * 2;
    warning_zone.scale.y = warning_distance * 2;
    warning_zone.color.r = 1.0;
    warning_zone.color.g = 0.984;
    warning_zone.color.b = 0.0;
    warning_zone.color.a = 0.5;
    zones.markers.push_back(warning_zone);
  }

private:
  double warning_distance, danger_distance;
  bool is_first_msg;
  State curr_state, prev_state;
  visualization_msgs::msg::MarkerArray zones;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_stop_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr zones_pub;

  rclcpp_action::Client<twist_mux_msgs::action::JoyTurbo>::SharedPtr increase_speed_client;
  rclcpp_action::Client<twist_mux_msgs::action::JoyTurbo>::SharedPtr decrease_speed_client;

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
          zones.markers.at(0).color.a = 1;
          zones.markers.at(1).color.a = 1;
        }
        else if(curr_state == State::Warning)
        {
          is_safety_stop.data = false;
          decrease_speed_client->async_send_goal(twist_mux_msgs::action::JoyTurbo::Goal());
          zones.markers.at(0).color.a = 0.5;
          zones.markers.at(1).color.a = 1;
        }
        else if(curr_state == State::Free)
        {
          is_safety_stop.data = false;
          increase_speed_client->async_send_goal(twist_mux_msgs::action::JoyTurbo::Goal());
          zones.markers.at(0).color.a = 0.5;
          zones.markers.at(1).color.a = 0.5;
        }
        prev_state = curr_state;
        safety_stop_pub->publish(is_safety_stop);
      }

      if(is_first_msg)
      {
        for(auto &zone : zones.markers)
        {
          zone.header.frame_id = msg.header.frame_id;
        }
        is_first_msg = false;
      }

      zones_pub->publish(zones);
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
