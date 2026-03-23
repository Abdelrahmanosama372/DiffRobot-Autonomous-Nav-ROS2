#ifndef DIFFBOT_PLANNING__DIJKSTRA_PLANNER_HPP_
#define DIFFBOT_PLANNING__DIJKSTRA_PLANNER_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <utility>

namespace diffbot_planning {

struct GraphNode {
  int x;
  int y;
  int cost;
  std::shared_ptr<GraphNode> parent;

  GraphNode(int x_value, int y_value)
      : x(x_value), y(y_value), cost(0), parent(nullptr) {}

  GraphNode operator+(const std::pair<int, int> &offset) const {
    GraphNode next_node(x + offset.first, y + offset.second);
    next_node.parent = std::make_shared<GraphNode>(*this);
    next_node.cost = cost + 1;
    return next_node;
  }

  bool operator==(const GraphNode &other) const {
    return x == other.x && y == other.y;
  }

  bool operator<(const GraphNode &other) const { return cost > other.cost; }

  void setCost(int new_cost) { cost = new_cost; }
};

struct MapLocation {
  int x;
  int y;

  MapLocation(int _x, int _y) : x(_x), y(_y) {}
};

class DijkstraPlanner : public rclcpp::Node {
public:
  explicit DijkstraPlanner(const std::string &node_name = "dijkstra_planner");

private:
  MapLocation poseToMap(const geometry_msgs::msg::PoseStamped &pose) const;
  geometry_msgs::msg::PoseStamped mapToPose(const MapLocation &location) const;
  bool is_pose_on_map(const geometry_msgs::msg::PoseStamped &pose) const;

  void
  set_new_goal(const geometry_msgs::msg::PoseStamped::SharedPtr _goal_pose);
  void set_new_map(const nav_msgs::msg::OccupancyGrid::SharedPtr _map);
  void plan(const geometry_msgs::msg::PoseStamped &start,
            const geometry_msgs::msg::PoseStamped &goal);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr plan_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::msg::PoseStamped goal_pose_;
  nav_msgs::msg::OccupancyGrid map_;
};

} // namespace diffbot_planning

#endif // DIFFBOT_PLANNING__DIJKSTRA_PLANNER_HPP_
