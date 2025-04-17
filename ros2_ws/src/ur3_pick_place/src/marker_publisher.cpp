#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/color_rgba.hpp>

using namespace std::chrono_literals;

class MarkerPublisher : public rclcpp::Node
{
public:
  MarkerPublisher() : Node("marker_publisher")
  {
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/visualization_marker_array", 10);
    
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MarkerPublisher::timer_callback, this));
    
    // Object positions
    red_cube_pose_.position.x = 0.4;
    red_cube_pose_.position.y = 0.0;
    red_cube_pose_.position.z = 0.05;
    red_cube_pose_.orientation.w = 1.0;
    
    blue_cylinder_pose_.position.x = 0.4;
    blue_cylinder_pose_.position.y = 0.2;
    blue_cylinder_pose_.position.z = 0.05;
    blue_cylinder_pose_.orientation.w = 1.0;
    
    place_pose_.position.x = 0.4;
    place_pose_.position.y = -0.2;
    place_pose_.position.z = 0.05;
    place_pose_.orientation.w = 1.0;
    
    RCLCPP_INFO(this->get_logger(), "Marker publisher initialized");
  }

private:
  void timer_callback()
  {
    auto marker_array = visualization_msgs::msg::MarkerArray();
    
    // Red cube marker
    auto red_cube_marker = visualization_msgs::msg::Marker();
    red_cube_marker.header.frame_id = "base_link";
    red_cube_marker.header.stamp = this->now();
    red_cube_marker.ns = "objects";
    red_cube_marker.id = 1;
    red_cube_marker.type = visualization_msgs::msg::Marker::CUBE;
    red_cube_marker.action = visualization_msgs::msg::Marker::ADD;
    red_cube_marker.pose = red_cube_pose_;
    red_cube_marker.scale.x = 0.1;
    red_cube_marker.scale.y = 0.1;
    red_cube_marker.scale.z = 0.1;
    red_cube_marker.color.r = 1.0;
    red_cube_marker.color.g = 0.0;
    red_cube_marker.color.b = 0.0;
    red_cube_marker.color.a = 1.0;
    marker_array.markers.push_back(red_cube_marker);
    
    // Blue cylinder marker
    auto blue_cylinder_marker = visualization_msgs::msg::Marker();
    blue_cylinder_marker.header.frame_id = "base_link";
    blue_cylinder_marker.header.stamp = this->now();
    blue_cylinder_marker.ns = "objects";
    blue_cylinder_marker.id = 2;
    blue_cylinder_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    blue_cylinder_marker.action = visualization_msgs::msg::Marker::ADD;
    blue_cylinder_marker.pose = blue_cylinder_pose_;
    blue_cylinder_marker.scale.x = 0.1;
    blue_cylinder_marker.scale.y = 0.1;
    blue_cylinder_marker.scale.z = 0.1;
    blue_cylinder_marker.color.r = 0.0;
    blue_cylinder_marker.color.g = 0.0;
    blue_cylinder_marker.color.b = 1.0;
    blue_cylinder_marker.color.a = 1.0;
    marker_array.markers.push_back(blue_cylinder_marker);
    
    // Place location marker
    auto place_marker = visualization_msgs::msg::Marker();
    place_marker.header.frame_id = "base_link";
    place_marker.header.stamp = this->now();
    place_marker.ns = "objects";
    place_marker.id = 3;
    place_marker.type = visualization_msgs::msg::Marker::CUBE;
    place_marker.action = visualization_msgs::msg::Marker::ADD;
    place_marker.pose = place_pose_;
    place_marker.scale.x = 0.2;
    place_marker.scale.y = 0.2;
    place_marker.scale.z = 0.02;
    place_marker.color.r = 0.0;
    place_marker.color.g = 1.0;
    place_marker.color.b = 0.0;
    place_marker.color.a = 1.0;
    marker_array.markers.push_back(place_marker);
    
    // Ground plane marker
    auto ground_marker = visualization_msgs::msg::Marker();
    ground_marker.header.frame_id = "base_link";
    ground_marker.header.stamp = this->now();
    ground_marker.ns = "objects";
    ground_marker.id = 4;
    ground_marker.type = visualization_msgs::msg::Marker::CUBE;
    ground_marker.action = visualization_msgs::msg::Marker::ADD;
    ground_marker.pose.position.x = 0.0;
    ground_marker.pose.position.y = 0.0;
    ground_marker.pose.position.z = -0.01;
    ground_marker.pose.orientation.w = 1.0;
    ground_marker.scale.x = 2.0;
    ground_marker.scale.y = 2.0;
    ground_marker.scale.z = 0.01;
    ground_marker.color.r = 0.8;
    ground_marker.color.g = 0.8;
    ground_marker.color.b = 0.8;
    ground_marker.color.a = 0.5;
    marker_array.markers.push_back(ground_marker);
    
    publisher_->publish(marker_array);
    RCLCPP_INFO(this->get_logger(), "Published markers");
  }
  
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Pose red_cube_pose_;
  geometry_msgs::msg::Pose blue_cylinder_pose_;
  geometry_msgs::msg::Pose place_pose_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MarkerPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
