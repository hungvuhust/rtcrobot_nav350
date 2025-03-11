#pragma once

#include <algorithm>
#include <cstddef>
#include <ctime>
#include <glog/logging.h>
#include <iostream>
#include <iterator>

struct MyLogSink : google::LogSink {
  MyLogSink() : google::LogSink() {
    // add a custom log sink
    google::AddLogSink(this); // (1)!
  }
  ~MyLogSink() override {
    google::RemoveLogSink(this); // (3)!
  }
  const char *GetBasename(const char *filepath) {
    const char *base= strrchr(filepath, '/');
    return base ? (base + 1) : filepath;
  }
  void send(
      ::google::LogSeverity severity, const char *filename,
      const char *base_filename, int line, const struct std::tm *tm_time,
      const char *message, size_t message_len) override {

    const std::string message_string= ::google::LogSink::ToString(
        severity, GetBasename(filename), line, tm_time, message, message_len);

    const char *color;
    switch (severity) {
    case google::GLOG_INFO:
      color= "\033[1;32m"; // Green
      break;
    case google::GLOG_WARNING:
      color= "\033[1;33m"; // Yellow
      break;
    case google::GLOG_ERROR:
      color= "\033[1;31m"; // Red
      break;
    case google::GLOG_FATAL:
      color= "\033[1;41m"; // Red background
      break;
    default:
      color= "\033[0m"; // Reset
      break;
    }

    std::cout << color << "[" << google::GetLogSeverityName(severity) << "] ( "
              << base_filename << ':' << line << " ) -> ";
    std::copy_n(
        message, message_len, std::ostreambuf_iterator<char>{std::cout});
    std::cout << "\033[0m" << '\n'; // Reset color
  }
};