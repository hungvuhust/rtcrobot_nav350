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

using namespace sick_nav350;
using sensor_msgs::msg::LaserScan;
using geometry_msgs::msg::PoseStamped;
using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;
using geometry_msgs::msg::Pose2D;

namespace rtcrobot_nav350 {
class Nav350Node : public rclcpp::Node {
public:
  explicit Nav350Node(
      const rclcpp::NodeOptions &options= rclcpp::NodeOptions());
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

  // parameter
  std::string host_{"192.168.5.98"};
  int         port_{2112};
  std::string frame_id_{"nav_link"};
  std::string map_id_{"map"};

  // Timmer
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_boardcast_{nullptr};

  // Publisher
  rclcpp::Publisher<LaserScan>::SharedPtr   scan_pub_{nullptr};
  rclcpp::Publisher<PoseStamped>::SharedPtr pose_pub_{nullptr};
  rclcpp::Publisher<MarkerArray>::SharedPtr landmark_pub_{nullptr};
  // Bpoadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{nullptr};
  Pose2D                                         last_pose_data_;
};
} // namespace rtcrobot_nav350

#endif // __RTCROBOT_NAV350__HPP_