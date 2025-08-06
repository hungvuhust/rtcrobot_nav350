#ifndef __LOG_ROS_SINK__HPP_
#define __LOG_ROS_SINK__HPP_

#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>

class RosLogSink : public google::LogSink {
  rclcpp::Logger logger_{rclcpp::get_logger("NAV350 logger")};

public:
  RosLogSink() : google::LogSink() {
    google::AddLogSink(this);
  }
  ~RosLogSink() override {
    google::RemoveLogSink(this);
  }

  const char *GetBasename(const char *filepath) {
    const char *base = strrchr(filepath, '/');
    return base ? (base + 1) : filepath;
  }

  void send(google::LogSeverity severity,
            const char         *filename,
            const char         *base_filename,
            int                 line,
            const struct tm    *tm_time,
            const char         *message,
            size_t              message_len) override {
    (void)base_filename;
    const std::string message_string = ::google::LogSink::ToString(
      severity, GetBasename(filename), line, tm_time, message, message_len);

    switch (severity) {
      case ::google::GLOG_INFO:
        RCLCPP_INFO_STREAM(logger_, message_string);
        break;

      case ::google::GLOG_WARNING:
        RCLCPP_WARN_STREAM(logger_, message_string);
        break;

      case ::google::GLOG_ERROR:
        RCLCPP_ERROR_STREAM(logger_, message_string);
        break;

      case ::google::GLOG_FATAL:
        RCLCPP_FATAL_STREAM(logger_, message_string);
        break;
    }
  }
};

#endif  // __LOG_ROS_SINK__HPP_