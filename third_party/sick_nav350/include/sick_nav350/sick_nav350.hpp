#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <sick_nav350/epoll_tcp.hpp>
#include <sick_nav350/request_frame.hpp>
#include <sick_nav350/response_frame.hpp>
#include <sick_nav350/socket_tcp.hpp>
#include <sstream>
#include <string>
#include <sys/epoll.h>
#include <thread>

#include <sick_nav350/log_sink.hpp>
constexpr size_t BUFFER_SIZE= 1 << 16;
namespace sick_nav350 {

enum Nav350OperationMode {
  POWER_DOWN        = 0,
  STANDBY           = 1,
  MAPPING           = 2,
  LANDMARK_DETECTION= 3,
  NAVIGATION        = 4,
};

enum FlagSetParram {
  SET_ACCESS= 0,
  SET_MODE,
  SET_LAYER,
  READ_DEVICE_IDENT,
  REQUEST_POSITION_DATA,
  SET_LANDMARK_DATA_FORMAT,
  SET_SCAN_DATA_FORMAT,
};

class SickNav350 {
private:
  SocketTCP socket_;
  Epoll     epoll_;

  std::map<std::string, std::function<void(const std::vector<std::string> &)>>
      callback_map;

  // Data
  PoseData                  pose_data_;
  std::vector<LandmarkData> current_landmark_data_;
  std::vector<LandmarkData> current_layer_landmarks_;
  NavScan                   scan_data_;

  //   Thread
  std::shared_ptr<std::thread> thread_{nullptr};
  std::mutex                   mutex_;
  std::atomic<bool>            running_{true};
  std::atomic<int>             flag_set_parram_{0};

public:
  SickNav350(std::string ip, int port);
  SickNav350()                             = delete;
  SickNav350(const SickNav350 &)           = delete;
  SickNav350 &operator=(const SickNav350 &)= delete;
  SickNav350(SickNav350 &&)                = delete;
  SickNav350 &operator=(SickNav350 &&)     = delete;

  ~SickNav350();

  using SharedPtr= std::shared_ptr<SickNav350>;
  using UniquePtr= std::unique_ptr<SickNav350>;

public:
  void intitialize();
  void unintitialize();
  bool is_connect() { return socket_.is_connected(); }
  bool send_command(const std::string command, const std::string &params);

  PoseData get_pose_data() {
    std::lock_guard<std::mutex> lock(mutex_);
    return pose_data_;
  }
  std::vector<LandmarkData> get_current_landmark_data() {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_landmark_data_;
  }
  NavScan get_scan_data() {
    std::lock_guard<std::mutex> lock(mutex_);
    return scan_data_;
  }

  void print_data() {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_data_.print();
    for (const auto &landmark: current_landmark_data_) {
      landmark.print();
    }
    scan_data_.print();
  }

  bool set_access_mode(uint16_t mode, std::string password);
  bool set_layer(uint16_t layer);
  bool set_operation_mode(uint16_t mode);
  bool get_data_navigation();

private:
  void handle_set_access(const std::vector<std::string> &tokens);
  void handle_set_mode(const std::vector<std::string> &tokens);
  void handle_set_layer(const std::vector<std::string> &tokens);
  void handle_read_device_ident(const std::vector<std::string> &tokens);
  void handle_request_position_data(const std::vector<std::string> &tokens);
  bool check_response(
      const std::vector<std::string> &tokens, const std::string &command);

protected:
  void        thread_poll();
  std::string receive_frame();
  std::string create_frame(
      const std::string &commandKey, const std::string &params= "");

private:
  void process_pose_get_data(const std::vector<std::string> &tokens);
  NavScan
      parse_scan_data(const std::vector<std::string> &tokens, size_t &index);
  std::vector<LandmarkData> parse_landmark_data(
      const std::vector<std::string> &tokens, size_t &index);
  PoseData
       parse_pose_data(const std::vector<std::string> &tokens, size_t &index);
  bool is_number(const std::string &s);
  int  convert_hex_to_dec(const std::string &num);
  std::vector<std::string> split_string(const std::string &str, char delimiter);
};

} // namespace sick_nav350