#ifndef __RTCROBOT_NAV350__HPP_
#define __RTCROBOT_NAV350__HPP_

#include "rtcrobot_nav350/log_ros_sink.hpp"
#include "sick_nav350/sick_nav350.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <angles/angles.h>
#include <boost/iostreams/categories.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rtcrobot_interfaces/msg/nav350_data.hpp"

using namespace sick_nav350;
using geometry_msgs::msg::Pose2D;
using geometry_msgs::msg::PoseStamped;
using rtcrobot_interfaces::msg::Nav350Data;
using sensor_msgs::msg::LaserScan;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace rtcrobot_nav350 {
class Nav350Node : public rclcpp::Node {
public:
  explicit Nav350Node(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~Nav350Node();

protected:
  bool init_parameters();
  bool init_nav350();
  bool init_publisher();
  //   bool init_subscriber();
  //   bool init_service();
  bool init_timmer();

private:
  void timer_callback();
  void publish_scan();
  void publish_pose();
  void publish_landmark();
  void publish_tf();

private:
  SickNav350::SharedPtr nav350_{nullptr};

  std::map<uint16_t, std::string> reflector_type_map = {
      {0,    "initial positioning"},
      {1, "continuous positioning"},
      {2,    "virtual positioning"},
      {3,    "positioning stopped"},
      {4,       "position invalid"},
      {5,               "external"}
  };

  std::map<uint16_t, std::string> nav350_error_map = {
      {0,                     "no error"},
      {1,         "wrong operating mode"},
      {2, "asynchrony Method terminated"},
      {3,                 "invalid data"},
      {4,        "no position available"},
      {5,                      "timeout"},
      {6,        "method already active"},
      {7,                "general error"}
  };

  // parameter
  std::string host_{"192.168.5.98"};
  int         port_{2112};
  std::string frame_id_{"nav_link"};
  std::string map_id_{"map"};
  double      nav350_height_{1.9753};

  // Timmer
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_boardcast_{nullptr};

  // Publisher
  rclcpp::Publisher<LaserScan>::SharedPtr        scan_pub_{nullptr};
  rclcpp::Publisher<Nav350Data>::SharedPtr       pose_pub_{nullptr};
  rclcpp::Publisher<MarkerArray>::SharedPtr      landmark_pub_{nullptr};
  // Bpoadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{nullptr};
  Pose2D                                         last_pose_data_;
};
} // namespace rtcrobot_nav350

#endif // __RTCROBOT_NAV350__HPP_