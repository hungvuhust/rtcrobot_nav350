#include "sick_nav350/sick_nav350.hpp"
#include <cstdint>
#include <mutex>

// #define DEBUG 1

namespace sick_nav350 {

SickNav350::SickNav350(std::string ip, int port) : socket_(ip, port) {
  epoll_.addSocket(socket_.getFd(), EPOLLIN);

  if (!socket_.connect()) {
    LOG(ERROR) << "Error connecting to socket" << std::endl;
    return;
  }

  callback_map[SICK_NAV350_COMMANDS.at("SET_ACCESS").second] =
    std::bind(&SickNav350::handle_set_access, this, std::placeholders::_1);
  callback_map[SICK_NAV350_COMMANDS.at("SET_MODE").second] =
    std::bind(&SickNav350::handle_set_mode, this, std::placeholders::_1);
  callback_map[SICK_NAV350_COMMANDS.at("SET_LAYER").second] =
    std::bind(&SickNav350::handle_set_layer, this, std::placeholders::_1);
  callback_map[SICK_NAV350_COMMANDS.at("READ_DEVICE_IDENT").second] =
    std::bind(&SickNav350::handle_read_device_ident,
              this,
              std::placeholders::_1);
  callback_map[SICK_NAV350_COMMANDS.at("REQUEST_POSITION_DATA").second] =
    std::bind(&SickNav350::handle_request_position_data,
              this,
              std::placeholders::_1);
  callback_map[SICK_NAV350_COMMANDS.at("READ_LAYER").second] =
    std::bind(&SickNav350::handle_current_layer, this, std::placeholders::_1);

  for (const auto &command : callback_map) {
    LOG(INFO) << " Registering callback for: " << command.first << std::endl;
  }

  //   Thread
  thread_ = std::make_shared<std::thread>(&SickNav350::thread_poll, this);
}

SickNav350::~SickNav350() {
  // POWER DOWN
  set_access_mode(3, "F4724744");
  set_operation_mode(Nav350OperationMode::STANDBY);
  set_operation_mode(Nav350OperationMode::POWER_DOWN);
  //
  socket_.close();
  epoll_.close();

  running_.store(false);
  if (thread_) {
    if (thread_->joinable()) {
      thread_->join();
    }
  }
}

void SickNav350::thread_poll() {
  while (running_.load()) {
    // std::lock_guard<std::mutex> lock(mutex_);
    std::string response = receive_frame();
    if (response.empty()) {
      continue;
    }
  }
}

std::string SickNav350::receive_frame() {
  struct epoll_event events[1];  // Chỉ cần 1 socket, không cần nhiều event

  int nfds = epoll_wait(epoll_.getEpollFd(), events, 1, 5000);  // Chờ tối đa 5s
  if (nfds < 0) {
    perror(" epoll_wait failed");
    return "";
  } else if (nfds == 0) {
    LOG(ERROR) << "[WARNING] Timeout waiting for response\n";
    set_operation_mode(Nav350OperationMode::NAVIGATION);
    return "";
  }

  if (events[0].events & EPOLLIN) {  // Kiểm tra nếu có dữ liệu đến
    std::string response;
    char        buffer[BUFFER_SIZE] = {0};
    ssize_t     bytes_received      = 0;

    while (true) {
      bytes_received = read(socket_.getFd(), buffer, sizeof(buffer));
      if (bytes_received < 0) {
        perror(" Receive failed");
        return "";
      }
      response.append(buffer, bytes_received);

      if (response.find("\x03") != std::string::npos) {
        break;
      }
    }
#if DEBUG
    LOG(INFO) << "Received frame: " << response << std::endl;
#endif
    std::vector<std::string> tokens = split_string(response, ' ');
    if (callback_map.find(tokens[1].c_str()) != callback_map.end()) {
      std::lock_guard<std::mutex> lock(mutex_);
      callback_map[tokens[1]](tokens);
    }
    return response;
  }
  return "";
}

bool SickNav350::send_command(const std::string  command,
                              const std::string &params) {
  std::string          frame = create_frame(command, params);
  std::vector<uint8_t> frame_vector(frame.begin(), frame.end());
#if DEBUG
  LOG(INFO) << "Sending frame: " << frame << std::endl;
#endif
  return socket_.send(frame_vector);
}

bool SickNav350::check_response(const std::vector<std::string> &tokens,
                                const std::string              &command) {
  if (tokens.size() < 2) {
    LOG(ERROR) << " Invalid response format" << std::endl;
    return false;
  }
  if (tokens[1] == SICK_NAV350_COMMANDS.at(command).second) {
    return true;
  }
  return false;
}

void SickNav350::handle_set_access(const std::vector<std::string> &tokens) {
  if (check_response(tokens, "SET_ACCESS")) {
    if (tokens[2] == "1\x03") {
      LOG(INFO) << " Access mode success" << std::endl;
      flag_set_parram_ &= ~(1 << SET_ACCESS);
    } else {
      LOG(ERROR) << " Access mode failed" << std::endl;
    }
  } else {
    LOG(ERROR) << " Invalid response format" << std::endl;
  }
}
void SickNav350::handle_set_mode(const std::vector<std::string> &tokens) {
  if (check_response(tokens, "SET_MODE")) {
    if (!tokens[3].empty()) {
      LOG(INFO) << " Mode changed to " << tokens[3] << std::endl;
      flag_set_parram_ &= ~(1 << SET_MODE);
    }
  } else {
    LOG(ERROR) << " Invalid response format" << std::endl;
  }
}

void SickNav350::handle_set_layer(const std::vector<std::string> &tokens) {
  if (check_response(tokens, "SET_LAYER")) {
    LOG(INFO) << " Layer set to " << tokens[2] << std::endl;
    flag_set_parram_ &= ~(1 << SET_LAYER);
  } else {
    LOG(ERROR) << " Invalid response format" << std::endl;
  }
}

void SickNav350::handle_read_device_ident(
  const std::vector<std::string> &tokens) {
  if (check_response(tokens, "READ_DEVICE_IDENT")) {
    LOG(INFO) << " Device Ident: " << tokens[2] << std::endl;
  } else {
    LOG(ERROR) << " Invalid response format" << std::endl;
  }
}

void SickNav350::handle_request_position_data(
  const std::vector<std::string> &tokens) {
  if (check_response(tokens, "REQUEST_POSITION_DATA")) {
    process_pose_get_data(tokens);
  }
}

void SickNav350::process_pose_get_data(const std::vector<std::string> &tokens) {
  if (tokens.size() < 4 || !check_response(tokens, "REQUEST_POSITION_DATA")) {
    LOG(ERROR) << " Invalid response format????? " << tokens.size() << " "
               << std::endl;
    return;
  }
  int errorCode = convert_hex_to_dec(tokens[3]);
  if (errorCode != 0) {
    switch (errorCode) {
      case 1:
        LOG(ERROR) << "wrong operating mode: " << errorCode << std::endl;
        break;
      case 2:
        LOG(ERROR) << "asynchrony Method terminated: " << errorCode
                   << std::endl;
        break;
      case 3:
        LOG(ERROR) << "invalid data: " << errorCode << std::endl;
        break;
      case 4:
        LOG(ERROR) << "no position available: " << errorCode << std::endl;
        break;
      case 5:
        LOG(ERROR) << "time out: " << errorCode << std::endl;
        break;
      case 6:
        LOG(ERROR) << "method already active: " << errorCode << std::endl;
        break;
      case 7:
        LOG(ERROR) << "general error: " << errorCode << std::endl;
        break;
      default:
        LOG(ERROR) << "Doen't know error code: " << errorCode << std::endl;
        break;
    }
    return;
  }

  // LOG(INFO) << " Valid response received. Parsing data..." <<
  // std::endl;
  try {
    size_t   index        = 5;  // Start at Mask
    uint16_t mask         = convert_hex_to_dec(tokens[index++]);
    bool     is_pose_data = convert_hex_to_dec(tokens[index++]) == 1;
    if (is_pose_data) {
      pose_data_ = parse_pose_data(tokens, index);
    }

    switch (mask) {
      case 0:  // pose + ref
      {
        // Landmark Data
        bool is_landmark_data = convert_hex_to_dec(tokens[index++]) == 1;
        if (is_landmark_data) {
          current_landmark_data_ = parse_landmark_data(tokens, index);
        }
        break;
      }
      case 1:  // pose + scan
      {
        // Scan Data
        uint16_t type_scan_data = convert_hex_to_dec(tokens[index++]);
        if (type_scan_data != 0) {
          scan_data_ = parse_scan_data(tokens, index);
        }
        break;
      }
      case 2:  // pose + ref + scan
      {
        // Landmark Data
        bool is_landmark_data = convert_hex_to_dec(tokens[index++]) == 1;
        if (is_landmark_data) {
          current_landmark_data_ = parse_landmark_data(tokens, index);
        }
        // Scan Data
        uint16_t type_scan_data = convert_hex_to_dec(tokens[index++]);
        if (type_scan_data != 0) {
          scan_data_ = parse_scan_data(tokens, index);
        }
        break;
      }
      default:
        break;
    }

    flag_set_parram_ &= ~(1 << REQUEST_POSITION_DATA);
  } catch (const std::exception &e) {
    flag_set_parram_ &= ~(1 << REQUEST_POSITION_DATA);
    LOG(ERROR) << e.what() << std::endl;
  }
}

void SickNav350::handle_current_layer(const std::vector<std::string> &tokens) {
  if (check_response(tokens, "READ_LAYER")) {
    process_current_layer(tokens);
  }
}

void SickNav350::process_current_layer(const std::vector<std::string> &tokens) {
  if (tokens.size() < 3 || !check_response(tokens, "READ_LAYER")) {
    LOG(ERROR) << " Invalid response format" << std::endl;
    return;
  }
  current_layer_ = convert_hex_to_dec(tokens[2]);
  LOG(INFO) << " Current layer: " << current_layer_ << std::endl;
  flag_set_parram_ &= ~(1 << READ_LAYER);
}

PoseData SickNav350::parse_pose_data(const std::vector<std::string> &tokens,
                                     size_t                         &index) {
  if (index + 2 >= tokens.size()) {
    throw std::runtime_error(" PoseData missing fields");
  }
  PoseData pose;
  pose.is_pose_data = 1;
  if (pose.is_pose_data) {
    pose.x                = convert_hex_to_dec(tokens[index++]);
    pose.y                = convert_hex_to_dec(tokens[index++]);
    pose.phi              = convert_hex_to_dec(tokens[index++]);
    pose.is_opt_pose_data = convert_hex_to_dec(tokens[index++]) == 1;
    if (pose.is_opt_pose_data) {
      pose.optional_pose_data.output_mode = convert_hex_to_dec(tokens[index++]);
      pose.optional_pose_data.timestamp   = convert_hex_to_dec(tokens[index++]);
      pose.optional_pose_data.mean_dev    = convert_hex_to_dec(tokens[index++]);
      pose.optional_pose_data.nav_mode    = convert_hex_to_dec(tokens[index++]);
      pose.optional_pose_data.info_state  = convert_hex_to_dec(tokens[index++]);
      pose.optional_pose_data.num_reflector_used =
        convert_hex_to_dec(tokens[index++]);
    }
  }
  return pose;
}

// Xử lý dữ liệu LandmarkData
std::vector<LandmarkData> SickNav350::parse_landmark_data(
  const std::vector<std::string> &tokens,
  size_t                         &index) {
  if (index + 5 >= tokens.size()) {
    throw std::runtime_error(" LandmarkData missing fields");
  }
  // Landmark Filter
  uint16_t landmark_filter = convert_hex_to_dec(tokens[index++]);
  if (landmark_filter != 1) {
    throw std::runtime_error(" Invalid landmark filter");
  }
  std::vector<LandmarkData> landmarks;
  // get size of landmarks
  uint16_t                  size = convert_hex_to_dec(tokens[index++]);
  landmarks.reserve(size);
  for (uint16_t i = 0; i < size; i++) {
    LandmarkData landmark{};
    // cart or polar
    if (convert_hex_to_dec(tokens[index++]) == 1) {
      landmark.cart_data.x = convert_hex_to_dec(tokens[index++]);
      landmark.cart_data.y = convert_hex_to_dec(tokens[index++]);
    }
    if (convert_hex_to_dec(tokens[index++]) == 1) {
      landmark.polar_data.distance = convert_hex_to_dec(tokens[index++]);
      landmark.polar_data.phi      = convert_hex_to_dec(tokens[index++]);
    }
    landmark.is_opt_landmark_data = convert_hex_to_dec(tokens[index++]) == 1;
    if (landmark.is_opt_landmark_data) {
      landmark.optional_landmark_data.local_id =
        convert_hex_to_dec(tokens[index++]);
      landmark.optional_landmark_data.global_id =
        convert_hex_to_dec(tokens[index++]);
      landmark.optional_landmark_data.landmark_type =
        convert_hex_to_dec(tokens[index++]);
      landmark.optional_landmark_data.reflector_type =
        convert_hex_to_dec(tokens[index++]);
      landmark.optional_landmark_data.reserved =
        convert_hex_to_dec(tokens[index++]);
      landmark.optional_landmark_data.timestamp =
        convert_hex_to_dec(tokens[index++]);
      landmark.optional_landmark_data.size =
        convert_hex_to_dec(tokens[index++]);
      landmark.optional_landmark_data.num_hit =
        convert_hex_to_dec(tokens[index++]);
      landmark.optional_landmark_data.mean_amplitude =
        convert_hex_to_dec(tokens[index++]);
      landmark.optional_landmark_data.start_index =
        convert_hex_to_dec(tokens[index++]);
      landmark.optional_landmark_data.end_index =
        convert_hex_to_dec(tokens[index++]);
    }
    landmarks.push_back(landmark);
  }
  return landmarks;
}

// Xử lý dữ liệu ScanData
NavScan SickNav350::parse_scan_data(const std::vector<std::string> &tokens,
                                    size_t                         &index) {
  if (index + 4 >= tokens.size()) {
    throw std::runtime_error(" ScanData missing fields");
  }
  NavScan scan{};
  scan.scan_data.content_type = tokens[index++];

  if (scan.scan_data.content_type != "DIST1" and
      scan.scan_data.content_type != "ANGL1") {
    throw std::runtime_error(" Invalid content type of scan: " +
                             scan.scan_data.content_type);
  }

  scan.scan_data.scale_factor    = (double)convert_hex_to_dec(tokens[index++]);
  scan.scan_data.scale_offset    = (double)convert_hex_to_dec(tokens[index++]);
  scan.scan_data.start_angle     = convert_hex_to_dec(tokens[index++]);
  scan.scan_data.angular_step    = convert_hex_to_dec(tokens[index++]);
  scan.scan_data.timestamp_start = convert_hex_to_dec(tokens[index++]);
  scan.scan_data.num_datas       = convert_hex_to_dec(tokens[index++]);
  scan.scan_data.distance_data.reserve(scan.scan_data.num_datas);

  for (uint16_t i = 0; i < scan.scan_data.num_datas; i++) {
    scan.scan_data.distance_data.push_back(convert_hex_to_dec(tokens[index++]));
  }

  scan.is_remission_data = convert_hex_to_dec(tokens[index++]) == 1;
  if (scan.is_remission_data) {
    scan.remission_data.content_type = tokens[index++];
    if (scan.remission_data.content_type != "RSSI1") {
      throw std::runtime_error("Invalid content type of remission: " +
                               scan.remission_data.content_type);
    }

    scan.remission_data.scale_factor =
      (double)convert_hex_to_dec(tokens[index++]);
    scan.remission_data.scale_offset =
      (double)convert_hex_to_dec(tokens[index++]);
    scan.remission_data.start_angle     = convert_hex_to_dec(tokens[index++]);
    scan.remission_data.angular_step    = convert_hex_to_dec(tokens[index++]);
    scan.remission_data.timestamp_start = convert_hex_to_dec(tokens[index++]);
    scan.remission_data.num_datas       = convert_hex_to_dec(tokens[index++]);
    scan.remission_data.remission_data.reserve(scan.remission_data.num_datas);

    for (uint16_t i = 0; i < scan.remission_data.num_datas; i++) {
      scan.remission_data.remission_data.push_back(
        convert_hex_to_dec(tokens[index++]));
    }
  }

  return scan;
}

// Kiểm tra một chuỗi có phải là số không
bool SickNav350::is_number(const std::string &s) {
  return !s.empty() && std::all_of(s.begin(), s.end(), ::isdigit);
}

int SickNav350::convert_hex_to_dec(const std::string &num) {
  int suma = 0;
  for (size_t i = 0; i < num.length(); i++) {
    if (num[i] >= 65) {
      suma = suma * 16 + num[i] - 65 + 10;
    } else {
      suma = suma * 16 + num[i] - 48;
    }
  }
  return suma;
}

std::vector<std::string> SickNav350::split_string(const std::string &str,
                                                  char delimiter) {
  std::vector<std::string> tokens;
  std::stringstream        ss(str);
  std::string              token;
  while (std::getline(ss, token, delimiter)) {
    tokens.push_back(token);
  }

  return tokens;
}

std::string SickNav350::create_frame(const std::string &commandKey,
                                     const std::string &params) {
  if (SICK_NAV350_COMMANDS.find(commandKey) == SICK_NAV350_COMMANDS.end()) {
    return "Invalid Command!";
  }

  auto cmd = SICK_NAV350_COMMANDS.at(commandKey);

  // Frame bắt đầu với STX (0x02) và kết thúc với ETX (0x03)
  std::string frame = "\x02" + cmd.first + " " + cmd.second;

  if (!params.empty()) {
    frame += " " + params;
  }

  frame += "\x03";  // ETX

  // Debug

#if DEBUG
  std::cout << "[DEBUG]  ---------->Send: " << frame << std::endl;
#endif
  return frame;
}

bool SickNav350::set_access_mode(uint16_t mode, std::string password) {
  // Validate mode 2 or 3
  if (mode != 2 && mode != 3) {
    return false;
  }

  flag_set_parram_ |= (1 << SET_ACCESS);
  std::string mode_str = std::to_string(mode);
  send_command("SET_ACCESS", mode_str + " " + password);

  // timeout
  auto start = std::chrono::system_clock::now();
  while (flag_set_parram_ & (1 << SET_ACCESS)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto end = std::chrono::system_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(end - start).count() >
        5) {
      flag_set_parram_ &= ~(1 << SET_ACCESS);
      return false;
    }
  }
  return true;
}

bool SickNav350::set_layer(uint16_t layer) {
  flag_set_parram_ |= (1 << SET_LAYER);
  // Validate layer 0-15
  if (layer > 15) {
    return false;
  }
  std::string layer_str = std::to_string(layer);
  send_command("SET_LAYER", layer_str);

  // timeout
  auto start = std::chrono::system_clock::now();
  while (flag_set_parram_ & (1 << SET_LAYER)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto end = std::chrono::system_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(end - start).count() >
        5) {
      flag_set_parram_ &= ~(1 << SET_LAYER);
      return false;
    }
  }
  return true;
}

bool SickNav350::set_operation_mode(uint16_t mode) {
  // Validate mode 0-4
  if (mode > 4) {
    return false;
  }
  flag_set_parram_ |= (1 << SET_MODE);
  std::string mode_str = std::to_string(mode);
  send_command("SET_MODE", mode_str);

  // timeout
  auto start = std::chrono::system_clock::now();
  while (flag_set_parram_ & (1 << SET_MODE)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto end = std::chrono::system_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(end - start).count() >
        5) {
      flag_set_parram_ &= ~(1 << SET_MODE);
      return false;
    }
  }
  return true;
}

bool SickNav350::get_data_navigation() {
  flag_set_parram_ |= (1 << REQUEST_POSITION_DATA);
  // Request data
  send_command("REQUEST_POSITION_DATA", "1 2");
  // timeout
  auto start = std::chrono::system_clock::now();
  while (flag_set_parram_ & (1 << REQUEST_POSITION_DATA)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto end = std::chrono::system_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count() > 500) {
      flag_set_parram_ &= ~(1 << REQUEST_POSITION_DATA);
      return false;
    }
  }
  return true;
}

bool SickNav350::send_get_current_layer() {
  flag_set_parram_ |= (1 << READ_LAYER);
  send_command("READ_LAYER", "");
  // timeout
  auto start = std::chrono::system_clock::now();
  while (flag_set_parram_ & (1 << READ_LAYER)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto end = std::chrono::system_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count() > 500) {
      flag_set_parram_ &= ~(1 << READ_LAYER);
      return false;
    }
  }
  return true;
}

void SickNav350::intitialize() {
  // Login
  set_access_mode(3, "F4724744");
  // Change mode to standby 1
  set_operation_mode(Nav350OperationMode::STANDBY);
  // Set Landmark Data Format
  send_command("SET_LANDMARK_DATA_FORMAT", "0 1 1");
  // Set Scan Data Format
  send_command("SET_SCAN_DATA_FORMAT", "1 0");
  // Set Pose Data Format
  send_command("SET_POS_DATA_FORMAT", "1 1");
  // Set Size of Reflector
  send_command("SET_REFLECTOR_SIZE", "28");
}

void SickNav350::unintitialize() {
  // Login
  set_access_mode(3, "F4724744");
  // Change mode to standby 1
  set_operation_mode(Nav350OperationMode::STANDBY);
  // POWER DOWN
  set_operation_mode(Nav350OperationMode::POWER_DOWN);
}

}  // namespace sick_nav350