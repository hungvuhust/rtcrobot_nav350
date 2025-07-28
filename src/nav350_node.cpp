#include "rtcrobot_nav350/nav350_node.hpp"
#include <memory>
#include <rclcpp/parameter_value.hpp>

namespace rtcrobot_nav350 {

Nav350Node::Nav350Node(const rclcpp::NodeOptions &)
    : rclcpp::Node(
          "nav350_node", rclcpp::NodeOptions().use_intra_process_comms(true)) {
  RCLCPP_INFO(get_logger(), "Nav350Node has been started.");
  if (!init_parameters()) {
    LOG(ERROR) << "Failed to initialize parameters.";
  }

  if (!init_nav350()) {
    LOG(ERROR) << "Failed to initialize nav350.";
  }

  if (!init_publisher()) {
    LOG(ERROR) << "Failed to initialize publisher.";
  }

  //   if (!init_subscriber()) {
  //     LOG(ERROR) << "Failed to initialize subscriber.";
  //   }

  //   if (!init_service()) {
  //     LOG(ERROR) << "Failed to initialize service.";
  //   }

  if (!init_timmer()) {
    LOG(ERROR) << "Failed to initialize timmer.";
  }
}

Nav350Node::~Nav350Node() {

  if (nav350_) {
    nav350_->unintitialize();
  }
}

bool Nav350Node::init_parameters() {
  try {
    // DECLARE PARAMETER
    declare_parameter("ip_address", rclcpp::ParameterValue("192.168.0.1"));
    declare_parameter("port", rclcpp::ParameterValue(1111));
    declare_parameter("nav350_frame_id", rclcpp::ParameterValue("base_link"));
    declare_parameter("map_frame_id", rclcpp::ParameterValue("map"));
    // GET PARAMETER
    get_parameter("ip_address", host_);
    get_parameter("port", port_);
    get_parameter("nav350_frame_id", frame_id_);
    get_parameter("map_frame_id", map_id_);
    // LOG
    LOG(INFO) << "ip_address: " << host_;
    LOG(INFO) << "port: " << port_;
    LOG(INFO) << "nav350_frame_id: " << frame_id_;
    LOG(INFO) << "map_frame_id: " << map_id_;
  } catch (const std::exception &e) {
    LOG(ERROR) << "Exception: " << e.what();
    return false;
  }
  return true;
}
bool Nav350Node::init_nav350() {
  try {
    nav350_= std::make_shared<SickNav350>(host_, port_);
    if (!nav350_->is_connect()) {
      LOG(ERROR) << "Error connecting to device" << std::endl;
      return 1;
    }
    nav350_->intitialize();
    // Set layer 0
    nav350_->set_layer(0);
    // Change mode to navigation
    nav350_->set_operation_mode(Nav350OperationMode::NAVIGATION);
  } catch (const std::exception &e) {
    LOG(ERROR) << "Exception: " << e.what();
    return false;
  }
  return true;
}

bool Nav350Node::init_publisher() {
  try {
    scan_pub_    = create_publisher<LaserScan>("/nav/scan", 5);
    pose_pub_    = create_publisher<PoseStamped>("/nav/pose", 10);
    landmark_pub_= create_publisher<MarkerArray>("/nav/landmark", 10);
  } catch (const std::exception &e) {
    LOG(ERROR) << "Exception: " << e.what();
    return false;
  }
  return true;
}
// bool Nav350Node::init_subscriber() {}
// bool Nav350Node::init_service() {}
bool Nav350Node::init_timmer() {
  try {
    tf_broadcaster_= std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_         = create_wall_timer(
                 std::chrono::milliseconds(100),
                 std::bind(&Nav350Node::timer_callback, this));
  } catch (const std::exception &e) {
    LOG(ERROR) << "Exception: " << e.what();
    return false;
  }
  return true;
}

void Nav350Node::timer_callback() {
  if (nav350_) {
    nav350_->get_data_navigation();
    publish_scan();
    publish_pose();
    publish_landmark();
    publish_tf();
  }
}

void Nav350Node::publish_scan() {
  try {
    // Get data of scan
    auto scan_data= nav350_->get_scan_data();
    // Create message of scan
    auto scan_msg      = std::make_unique<LaserScan>();
    scan_msg->angle_min= -M_PI;
    scan_msg->angle_max= M_PI;
    scan_msg->range_min= 0.05;
    scan_msg->range_max= 70.0;

    scan_msg->angle_increment= 2 * M_PI / scan_data.scan_data.num_datas;
    scan_msg->ranges.resize(scan_data.scan_data.num_datas);
    if (scan_data.is_remission_data) {
      scan_msg->intensities.resize(scan_data.remission_data.num_datas);
      for (int i= 0; i < scan_data.scan_data.num_datas; i++) {
        scan_msg->ranges[i]     = scan_data.scan_data.distance_data[i] * 0.001;
        scan_msg->intensities[i]= scan_data.remission_data.remission_data[i];
      }
    } else {
      for (int i= 0; i < scan_data.scan_data.num_datas; i++) {
        scan_msg->ranges[i]= scan_data.scan_data.distance_data[i] * 0.001;
      }
    }

    scan_msg->header.stamp   = now();
    scan_msg->header.frame_id= frame_id_;
    // Publish message of scan
    scan_pub_->publish(std::move(scan_msg));
  } catch (const std::exception &e) {
    LOG(ERROR) << "Exception: " << e.what();
  }
}

void Nav350Node::publish_pose() {
  try {
    // Get data of pose
    auto pose_data       = nav350_->get_pose_data();
    last_pose_data_.theta= (pose_data.phi * M_PI / 180.0 * 0.001) - M_PI;
    // Rotate x, y by -M_PI
    last_pose_data_.x= pose_data.x * 0.001;
    last_pose_data_.y= pose_data.y * 0.001;
    // rotate_x_yby_angle_offset(last_pose_data_.x, last_pose_data_.y);
    // Create message of pose
    auto pose_msg            = std::make_unique<PoseStamped>();
    pose_msg->header.stamp   = now();
    pose_msg->header.frame_id= frame_id_;
    pose_msg->pose.position.x= last_pose_data_.x;
    pose_msg->pose.position.y= last_pose_data_.y;
    pose_msg->pose.position.z= 2.6;
    // convert phi to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, last_pose_data_.theta);
    pose_msg->pose.orientation.x= q.x();
    pose_msg->pose.orientation.y= q.y();
    pose_msg->pose.orientation.z= q.z();
    pose_msg->pose.orientation.w= q.w();
    // Publish message of pose
    pose_pub_->publish(std::move(pose_msg));
  } catch (const std::exception &e) {
    LOG(ERROR) << "Exception: " << e.what();
  }
}

void Nav350Node::publish_landmark() {
  try {
    // Get data of landmark
    auto landmark_data= nav350_->get_current_landmark_data();
    // Create message of landmark
    auto landmark_msg= std::make_unique<MarkerArray>();
    if (landmark_data.size() > 0) {
      for (auto &landmark: landmark_data) {
        Marker marker;
        marker.header.frame_id= frame_id_;
        marker.header.stamp   = now();
        marker.ns             = "landmark";
        if (landmark.is_opt_landmark_data) {
          marker.id= landmark.optional_landmark_data.local_id;
          if (landmark.optional_landmark_data.global_id != 65535) {
            // Landmark Known
            marker.color.r= 0.0;
            marker.color.g= 1.0;
            marker.color.b= 0.0;
            marker.color.a= 1.0;
            marker.text   = landmark.optional_landmark_data.local_id;
          } else {
            // Landmark Unknown
            marker.color.r= 1.0;
            marker.color.g= 0.0;
            marker.color.b= 0.0;
            marker.color.a= 1.0;
            marker.text   = "Unknown";
          }
        } else {
          // Landmark Unknown
          marker.color.r= 1.0;
          marker.color.g= 0.0;
          marker.color.b= 0.0;
          marker.color.a= 1.0;
          marker.text   = "Unknown";
        }
        marker.type              = Marker::SPHERE;
        marker.action            = Marker::ADD;
        marker.pose.position.x   = -landmark.cart_data.x * 0.001;
        marker.pose.position.y   = -landmark.cart_data.y * 0.001;
        marker.pose.orientation.w= 1.0;
        marker.scale.x           = 0.5;
        marker.scale.y           = 0.5;
        marker.scale.z           = 0.5;
        landmark_msg->markers.push_back(marker);
      }
    }
    // Publish message of landmark
    landmark_pub_->publish(std::move(landmark_msg));
  } catch (const std::exception &e) {
    LOG(ERROR) << "Exception: " << e.what();
  }
}

void Nav350Node::publish_tf() {
  try {
    // Get data of pose
    // Create message of tf
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp           = now();
    transform.header.frame_id        = map_id_;
    transform.child_frame_id         = frame_id_;
    transform.transform.translation.x= last_pose_data_.x;
    transform.transform.translation.y= last_pose_data_.y;
    transform.transform.translation.z= 2.6;
    // convert phi to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, last_pose_data_.theta);
    transform.transform.rotation.x= q.x();
    transform.transform.rotation.y= q.y();
    transform.transform.rotation.z= q.z();
    transform.transform.rotation.w= q.w();
    // Publish message of tf
    tf_broadcaster_->sendTransform(transform);
  } catch (const std::exception &e) {
    LOG(ERROR) << "Exception: " << e.what();
  }
}

} // namespace rtcrobot_nav350
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rtcrobot_nav350::Nav350Node);

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  RosLogSink ros_log_sink;
  rclcpp::spin(std::make_shared<rtcrobot_nav350::Nav350Node>());
  google::ShutdownGoogleLogging();
  rclcpp::shutdown();
  return 0;
}