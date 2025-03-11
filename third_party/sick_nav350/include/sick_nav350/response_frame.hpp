#include <algorithm>
#include <cstdint>
#include <iostream>
#include <optional>
#include <sick_nav350/log_sink.hpp>
#include <sstream>
#include <string>
#include <vector>

namespace sick_nav350 {

struct PoseData {
  bool is_pose_data;
  int  x, y, phi;
  bool is_opt_pose_data;
  struct OptionalPoseData {
    uint16_t output_mode;
    uint32_t timestamp;
    int32_t  mean_dev;
    uint16_t nav_mode;
    uint32_t info_state;
    uint16_t num_reflector_used;
  } optional_pose_data;

  void print() const {
    if (is_pose_data) {
      LOG(INFO) << " Pose Data: X=" << (float)x / 1000.0
                << ", Y=" << (float)y / 1000.0
                << ", Phi=" << (float)phi / 1000.0 << std::endl;
    }
    if (is_opt_pose_data) {
      LOG(INFO) << " Optional Pose Data: Output Mode="
                << optional_pose_data.output_mode
                << ", Timestamp=" << optional_pose_data.timestamp
                << ", Mean Deviation=" << optional_pose_data.mean_dev
                << ", Nav Mode=" << optional_pose_data.nav_mode
                << ", Info State=" << optional_pose_data.info_state
                << ", Number of Reflectors Used="
                << optional_pose_data.num_reflector_used << std::endl;
    }
  }
};

struct LandmarkData {
  struct CartData {
    int x, y;
  } cart_data;
  struct PolarData {
    int distance, phi;
  } polar_data;
  bool is_opt_landmark_data;
  struct OptionalLandmarkData {
    uint16_t local_id;
    uint16_t global_id;
    uint16_t landmark_type;
    uint16_t reflector_type;
    uint16_t reserved;
    uint32_t timestamp;
    uint16_t size;
    uint16_t num_hit;
    uint16_t mean_amplitude;
    uint16_t start_index;
    uint16_t end_index;
  } optional_landmark_data;

  void print() const {
    if (cart_data.x != 0 && cart_data.y != 0) {
      LOG(INFO) << " Landmark Data: Cartesian X=" << (float)cart_data.x / 1000.0
                << ", Y=" << (float)cart_data.y / 1000.0 << std::endl;
    }
    if (polar_data.distance != 0 && polar_data.phi != 0) {
      LOG(INFO) << " Landmark Data: Polar Distance="
                << (float)polar_data.distance / 1000.0
                << ", Phi=" << (float)polar_data.phi / 1000.0 << std::endl;
    }
    if (is_opt_landmark_data) {
      LOG(INFO) << " Optional Landmark Data: Local ID="
                << optional_landmark_data.local_id
                << ", Global ID=" << optional_landmark_data.global_id
                << ", Landmark Type=" << optional_landmark_data.landmark_type
                << ", Reflector Type=" << optional_landmark_data.reflector_type
                << ", Reserved=" << optional_landmark_data.reserved
                << ", Timestamp=" << optional_landmark_data.timestamp
                << ", Size=" << optional_landmark_data.size
                << ", Number of Hit=" << optional_landmark_data.num_hit
                << ", Mean Amplitude=" << optional_landmark_data.mean_amplitude
                << ", Start Index=" << optional_landmark_data.start_index
                << ", End Index=" << optional_landmark_data.end_index
                << std::endl;
    }
  }
};
struct NavScan {
  struct ScanData {
    std::string           content_type; // DIST1 or ANGL1
    double                scale_factor;
    double                scale_offset;
    int32_t               start_angle;
    uint16_t              angular_step;
    uint32_t              timestamp_start;
    uint16_t              num_datas;
    std::vector<uint32_t> distance_data;
  } scan_data;
  uint16_t is_remission_data;
  struct RemissionData {
    std::string           content_type;
    double                scale_factor;
    double                scale_offset;
    int32_t               start_angle;
    uint16_t              angular_step;
    uint32_t              timestamp_start;
    uint16_t              num_datas;
    std::vector<uint32_t> remission_data;
  } remission_data;

  void print() const {
    LOG(INFO) << "------------------------------------------" << std::endl;

    LOG(INFO) << " Scan Data: Content Type=" << scan_data.content_type
              << ", Scale Factor=" << scan_data.scale_factor
              << ", Scale Offset=" << scan_data.scale_offset
              << ", Start Angle=" << scan_data.start_angle
              << ", Angular Step=" << scan_data.angular_step
              << ", Timestamp Start=" << scan_data.timestamp_start
              << ", Number of Datas=" << scan_data.num_datas << std::endl;
    if (is_remission_data) {
      LOG(INFO) << " Amplitude Data: " << remission_data.content_type
                << ", Scale Factor=" << remission_data.scale_factor
                << ", Scale Offset=" << remission_data.scale_offset
                << ", Start Angle=" << remission_data.start_angle
                << ", Angular Step=" << remission_data.angular_step
                << ", Timestamp Start=" << remission_data.timestamp_start
                << ", Number of Datas=" << remission_data.num_datas
                << std::endl;
    }
  }
};

} // namespace sick_nav350