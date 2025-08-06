#include "rtcrobot_nav350/nav350_node.hpp"
#include <csignal>
#include <memory>
#include <rclcpp/parameter_value.hpp>
#include <signal.h>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
namespace rtcrobot_nav350 {

Nav350Node::Nav350Node(const rclcpp::NodeOptions &)
  : rclcpp::Node("nav350_node",
                 rclcpp::NodeOptions().use_intra_process_comms(true)) {
  last_log_time_ = std::chrono::system_clock::now();
  RCLCPP_INFO(get_logger(), "Nav350Node has been started.");

  // Khởi tạo logger trước tiên
  init_logger();

  if (!init_parameters()) {
    LOG(ERROR) << "Failed to initialize parameters.";
    throw std::runtime_error("Failed to initialize parameters");
  }

  if (!init_nav350()) {
    LOG(ERROR) << "Failed to initialize nav350.";
    RCLCPP_ERROR(get_logger(),
                 "Nav350 initialization failed, but continuing...");
    // Không throw exception để cho phép node chạy và retry sau
  }

  if (!init_publisher()) {
    LOG(ERROR) << "Failed to initialize publisher.";
    throw std::runtime_error("Failed to initialize publisher");
  }

  //   if (!init_subscriber()) {
  //     LOG(ERROR) << "Failed to initialize subscriber.";
  //   }

  if (!init_service()) {
    LOG(ERROR) << "Failed to initialize service.";
    throw std::runtime_error("Failed to initialize service");
  }

  if (!init_timmer()) {
    LOG(ERROR) << "Failed to initialize timmer.";
    throw std::runtime_error("Failed to initialize timer");
  }

  is_setup_done_ = true;
  RCLCPP_INFO(get_logger(), "Nav350Node initialization completed.");
}

Nav350Node::~Nav350Node() {
  if (nav350_) {
    nav350_->unintitialize();
  }
}

void Nav350Node::init_logger() {
  try {
    logger_ = spdlog::rotating_logger_mt("nav350_node",
                                         "/home/rtc/nav350_node.log",
                                         1024 * 1024 * 5,
                                         3);

    logger_->set_level(spdlog::level::info);
    logger_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%L%$] %v");

    RCLCPP_INFO(
      get_logger(),
      "Logger đã được khởi tạo thành công: /home/rtc/nav350_node.log");
    logger_->info("Nav350 Logger started successfully");
  } catch (const std::exception &e) {
    LOG(ERROR) << "Exception during logger initialization: " << e.what();
    RCLCPP_ERROR(get_logger(), "Không thể khởi tạo logger: %s", e.what());
    logger_ = nullptr;  // Đảm bảo logger_ là nullptr nếu init thất bại
  }
}

bool Nav350Node::init_parameters() {
  try {
    // DECLARE PARAMETER
    declare_parameter("ip_address", rclcpp::ParameterValue("192.168.0.1"));
    declare_parameter("port", rclcpp::ParameterValue(1111));
    declare_parameter("nav350_frame_id", rclcpp::ParameterValue("base_link"));
    declare_parameter("map_frame_id", rclcpp::ParameterValue("map"));
    declare_parameter("nav350_height", rclcpp::ParameterValue(1.9753));
    // GET PARAMETER
    get_parameter("ip_address", host_);
    get_parameter("port", port_);
    get_parameter("nav350_frame_id", frame_id_);
    get_parameter("map_frame_id", map_id_);
    get_parameter("nav350_height", nav350_height_);
    // LOG
    LOG(INFO) << "ip_address: " << host_;
    LOG(INFO) << "port: " << port_;
    LOG(INFO) << "nav350_frame_id: " << frame_id_;
    LOG(INFO) << "map_frame_id: " << map_id_;
    LOG(INFO) << "nav350_height: " << nav350_height_;
  } catch (const std::exception &e) {
    LOG(ERROR) << "Exception: " << e.what();
    return false;
  }
  return true;
}

// Template implementation for retry operation
template <typename Func>
bool Nav350Node::retry_operation(Func               operation,
                                 const std::string &operation_name,
                                 int                max_retries) {
  for (int attempt = 1; attempt <= max_retries; ++attempt) {
    try {
      RCLCPP_INFO(get_logger(),
                  "Thử %s - lần thứ %d/%d",
                  operation_name.c_str(),
                  attempt,
                  max_retries);

      if (operation()) {
        RCLCPP_INFO(get_logger(),
                    "%s thành công ở lần thứ %d",
                    operation_name.c_str(),
                    attempt);
        return true;
      }

      if (attempt < max_retries) {
        RCLCPP_WARN(get_logger(),
                    "%s thất bại lần thứ %d, đang retry sau 1 giây...",
                    operation_name.c_str(),
                    attempt);
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(),
                   "Exception trong %s lần thứ %d: %s",
                   operation_name.c_str(),
                   attempt,
                   e.what());
      if (attempt < max_retries) {
        RCLCPP_WARN(get_logger(), "Đang retry sau 1 giây...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }
  }

  RCLCPP_ERROR(get_logger(),
               "%s thất bại sau %d lần thử",
               operation_name.c_str(),
               max_retries);
  return false;
}

bool Nav350Node::init_nav350() {
  // Retry connection với Nav350
  auto connect_operation = [this]() -> bool {
    try {
      nav350_ = std::make_shared<SickNav350>(host_, port_);
      if (!nav350_->is_connect()) {
        LOG(ERROR) << "Error connecting to device";
        return false;
      }
      return true;
    } catch (const std::exception &e) {
      LOG(ERROR) << "Exception during connection: " << e.what();
      return false;
    }
  };

  if (!retry_operation(connect_operation, "kết nối Nav350", 3)) {
    return false;
  }

  // Retry initialization
  auto init_operation = [this]() -> bool {
    try {
      nav350_->intitialize();
      return true;
    } catch (const std::exception &e) {
      LOG(ERROR) << "Exception during initialization: " << e.what();
      return false;
    }
  };

  if (!retry_operation(init_operation, "khởi tạo Nav350", 3)) {
    return false;
  }

  // Retry set layer
  auto set_layer_operation = [this]() -> bool {
    try {
      nav350_->set_layer(0);
      return true;
    } catch (const std::exception &e) {
      LOG(ERROR) << "Exception during set layer: " << e.what();
      return false;
    }
  };

  if (!retry_operation(set_layer_operation, "thiết lập layer", 3)) {
    RCLCPP_WARN(get_logger(),
                "Không thể thiết lập layer, tiếp tục với cấu hình mặc định");
  }

  // Retry set operation mode
  auto set_mode_operation = [this]() -> bool {
    try {
      nav350_->set_operation_mode(Nav350OperationMode::NAVIGATION);
      return true;
    } catch (const std::exception &e) {
      LOG(ERROR) << "Exception during set operation mode: " << e.what();
      return false;
    }
  };

  if (!retry_operation(set_mode_operation, "thiết lập chế độ navigation", 3)) {
    RCLCPP_WARN(
      get_logger(),
      "Không thể thiết lập chế độ navigation, tiếp tục với cấu hình mặc định");
  }

  return true;
}

bool Nav350Node::init_publisher() {
  try {
    scan_pub_     = create_publisher<LaserScan>("/nav/scan", 5);
    pose_pub_     = create_publisher<Nav350Data>("/nav/pose", 10);
    landmark_pub_ = create_publisher<MarkerArray>("/nav/landmark", 10);
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
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_          = create_wall_timer(std::chrono::milliseconds(100),
                               std::bind(&Nav350Node::timer_callback, this));
  } catch (const std::exception &e) {
    LOG(ERROR) << "Exception: " << e.what();
    return false;
  }
  return true;
}

bool Nav350Node::init_service() {
  try {
    switch_map_srv_ =
      create_service<SwitchMap>("/rtcservice/switch_map",
                                std::bind(&Nav350Node::handle_switch_map,
                                          this,
                                          std::placeholders::_1,
                                          std::placeholders::_2));
  } catch (const std::exception &e) {
    LOG(ERROR) << "Exception: " << e.what();
    return false;
  }
  return true;
}

void Nav350Node::timer_callback() {
  if (!is_setup_done_) {
    return;
  }

  if (!nav350_) {
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         10000,
                         "Nav350 device chưa được khởi tạo");
    return;
  }

  // Đơn giản hóa: không retry trong timer để tránh blocking
  // Timer chạy với tần số cao (10Hz), nên việc miss một vài lần là ok
  try {
    nav350_->get_data_navigation();
    publish_scan();
    publish_pose();
    publish_landmark();
    publish_tf();
  } catch (const std::exception &e) {
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         5000,
                         "Lỗi khi lấy dữ liệu navigation: %s",
                         e.what());

    // Log vào file nếu logger có sẵn
    if (logger_) {
      logger_->warn("Navigation data error: {}", e.what());
    }
  }
}

void Nav350Node::publish_scan() {
  try {
    // Get data of scan
    auto scan_data      = nav350_->get_scan_data();
    // Create message of scan
    auto scan_msg       = std::make_unique<LaserScan>();
    scan_msg->angle_min = -M_PI;
    scan_msg->angle_max = M_PI;
    scan_msg->range_min = 0.05;
    scan_msg->range_max = 70.0;

    scan_msg->angle_increment = 2 * M_PI / scan_data.scan_data.num_datas;
    scan_msg->ranges.resize(scan_data.scan_data.num_datas);
    if (scan_data.is_remission_data) {
      scan_msg->intensities.resize(scan_data.remission_data.num_datas);
      for (int i = 0; i < scan_data.scan_data.num_datas; i++) {
        scan_msg->ranges[i]      = scan_data.scan_data.distance_data[i] * 0.001;
        scan_msg->intensities[i] = scan_data.remission_data.remission_data[i];
      }
    } else {
      for (int i = 0; i < scan_data.scan_data.num_datas; i++) {
        scan_msg->ranges[i] = scan_data.scan_data.distance_data[i] * 0.001;
      }
    }

    scan_msg->header.stamp    = now();
    scan_msg->header.frame_id = frame_id_;
    // Publish message of scan
    scan_pub_->publish(std::move(scan_msg));
  } catch (const std::exception &e) {
    LOG(ERROR) << "Exception: " << e.what();
  }
}

void Nav350Node::publish_pose() {
  try {
    // Get data of pose
    auto              pose_data = nav350_->get_pose_data();
    std::stringstream ss;

    last_pose_data_.theta = (pose_data.phi * M_PI / 180.0 * 0.001);
    if (last_pose_data_.theta > M_PI) {
      last_pose_data_.theta -= 2 * M_PI;
    }
    // Rotate x, y by -M_PI
    last_pose_data_.x = -pose_data.x * 0.001;
    last_pose_data_.y = -pose_data.y * 0.001;
    // rotate_x_yby_angle_offset(last_pose_data_.x, last_pose_data_.y);
    // Create message of pose
    auto pose_msg     = std::make_unique<Nav350Data>();
    pose_msg->x       = last_pose_data_.x;
    pose_msg->y       = last_pose_data_.y;
    pose_msg->theta   = last_pose_data_.theta;

    ss << "Pose data: (" << last_pose_data_.x << ", " << last_pose_data_.y
       << ", " << last_pose_data_.theta << ")";

    // Opt
    if (pose_data.is_opt_pose_data) {
      pose_msg->reflector = pose_data.optional_pose_data.num_reflector_used;
      pose_msg->mode =
        reflector_type_map[pose_data.optional_pose_data.nav_mode];
      pose_msg->error = 0;

      ss << ", Number of reflector used: "
         << pose_data.optional_pose_data.num_reflector_used;
      ss << ", Mean Dev: " << pose_data.optional_pose_data.mean_dev;
    }

    // Publish message of pose
    pose_pub_->publish(std::move(pose_msg));

    // Log every 300ms (chỉ khi logger đã được khởi tạo)
    if (logger_ && std::chrono::system_clock::now() - last_log_time_ > 300ms) {
      logger_->info(ss.str());
      last_log_time_ = std::chrono::system_clock::now();
    }
  } catch (const std::exception &e) {
    LOG(ERROR) << "Exception: " << e.what();
  }
}

void Nav350Node::publish_landmark() {
  try {
    // Get data of landmark
    auto landmark_data = nav350_->get_current_landmark_data();
    // Create message of landmark
    auto landmark_msg  = std::make_unique<MarkerArray>();
    if (landmark_data.size() > 0) {
      for (auto &landmark : landmark_data) {
        Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp    = now();
        marker.ns              = "landmark";
        if (landmark.is_opt_landmark_data) {
          marker.id = landmark.optional_landmark_data.local_id;
          if (landmark.optional_landmark_data.global_id != 65535) {
            // Landmark Known
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.text    = landmark.optional_landmark_data.local_id;
          } else {
            // Landmark Unknown
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.text    = "Unknown";
          }
        } else {
          // Landmark Unknown
          marker.color.r = 1.0;
          marker.color.g = 0.0;
          marker.color.b = 0.0;
          marker.color.a = 1.0;
          marker.text    = "Unknown";
        }
        marker.type               = Marker::SPHERE;
        marker.action             = Marker::ADD;
        marker.pose.position.x    = -landmark.cart_data.x * 0.001;
        marker.pose.position.y    = -landmark.cart_data.y * 0.001;
        marker.pose.orientation.w = 1.0;
        marker.scale.x            = 0.5;
        marker.scale.y            = 0.5;
        marker.scale.z            = 0.5;
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
    transform.header.stamp            = now();
    transform.header.frame_id         = map_id_;
    transform.child_frame_id          = frame_id_;
    transform.transform.translation.x = last_pose_data_.x;
    transform.transform.translation.y = last_pose_data_.y;
    transform.transform.translation.z = nav350_height_;
    // convert phi to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, last_pose_data_.theta);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    // Publish message of tf
    tf_broadcaster_->sendTransform(transform);
  } catch (const std::exception &e) {
    LOG(ERROR) << "Exception: " << e.what();
  }
}

void Nav350Node::handle_switch_map(
  const std::shared_ptr<SwitchMap::Request> request,
  std::shared_ptr<SwitchMap::Response>      response) {
  try {
    if (!is_setup_done_) {
      response->status = false;
      RCLCPP_WARN(get_logger(), "Nav350 chưa sẵn sàng, không thể switch map");
      return;
    }

    if (!nav350_) {
      response->status = false;
      RCLCPP_ERROR(get_logger(), "Nav350 không được khởi tạo");
      return;
    }

    is_setup_done_ = false;
    RCLCPP_INFO(get_logger(),
                "Bắt đầu switch map sang layer %d",
                request->map_layer);

    // Retry operation để get current layer
    auto get_current_layer_operation = [this]() -> bool {
      return nav350_->send_get_current_layer();
    };

    if (!retry_operation(get_current_layer_operation, "lấy current layer", 3)) {
      response->status = false;
      is_setup_done_   = true;
      return;
    }

    // Kiểm tra nếu đã ở layer đúng
    if (nav350_->get_current_layer() == request->map_layer) {
      RCLCPP_INFO(get_logger(),
                  "Đã ở layer %d, không cần switch",
                  request->map_layer);
      response->status = true;
      is_setup_done_   = true;
      return;
    }

    // Retry operation để switch layer
    auto switch_layer_operation = [this, request]() -> bool {
      try {
        nav350_->set_layer(request->map_layer);

        // Thay vì sleep 1s, poll với timeout để tránh blocking lâu
        const int max_polls = 10;  // max 1 second (100ms * 10)
        for (int i = 0; i < max_polls; ++i) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));

          if (nav350_->send_get_current_layer()) {
            if (nav350_->get_current_layer() == request->map_layer) {
              return true;
            }
          }
        }

        // Final check sau khi poll
        if (nav350_->send_get_current_layer()) {
          return nav350_->get_current_layer() == request->map_layer;
        }
        return false;
      } catch (const std::exception &e) {
        LOG(ERROR) << "Exception during layer switch: " << e.what();
        return false;
      }
    };

    if (retry_operation(switch_layer_operation, "switch map layer", 3)) {
      RCLCPP_INFO(get_logger(),
                  "Switch map thành công sang layer %d",
                  request->map_layer);
      response->status = true;
    } else {
      RCLCPP_ERROR(get_logger(), "Switch map thất bại sau 3 lần thử");
      response->status = false;
    }

  } catch (const std::exception &e) {
    LOG(ERROR) << "Exception trong handle_switch_map: " << e.what();
    response->status = false;
  }

  is_setup_done_ = true;
}

}  // namespace rtcrobot_nav350
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rtcrobot_nav350::Nav350Node);

// Signal handler cho Ctrl+C
void signal_handler(int signal) {
  if (signal == SIGINT) {
    RCLCPP_INFO(rclcpp::get_logger("nav350_node"),
                "Nhận signal SIGINT (Ctrl+C), đang shutdown...");
    rclcpp::shutdown();
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Đăng ký signal handler cho SIGINT (Ctrl+C)
  signal(SIGINT, signal_handler);

  google::InitGoogleLogging(argv[0]);
  RosLogSink ros_log_sink;

  RCLCPP_INFO(rclcpp::get_logger("nav350_node"),
              "Nav350 Node đã khởi động. Nhấn Ctrl+C để thoát.");

  rclcpp::spin(std::make_shared<rtcrobot_nav350::Nav350Node>());

  RCLCPP_INFO(rclcpp::get_logger("nav350_node"),
              "Nav350 Node đã shutdown thành công.");
  google::ShutdownGoogleLogging();
  rclcpp::shutdown();
  return 0;
}