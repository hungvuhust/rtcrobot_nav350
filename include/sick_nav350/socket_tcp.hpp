#ifndef _SOCKET_TCP_HPP_
#define _SOCKET_TCP_HPP_

#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

class SocketTCP {
public:
  SocketTCP(const std::string &address, int port) {
    sockfd= socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
      std::cerr << "Error creating socket" << std::endl;
      exit(1);
    }

    memset(&serverAddr, 0, sizeof(serverAddr));

    serverAddr.sin_family= AF_INET;
    serverAddr.sin_port  = htons(port);

    inet_pton(AF_INET, address.c_str(), &serverAddr.sin_addr);
  }

  ~SocketTCP() { close(); }

  bool connect() {
    return ::connect(
               sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) !=
           -1;
  }

  bool is_connected() {
    int       error = 0;
    socklen_t len   = sizeof(error);
    int       retval= getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &error, &len);
    return (retval == 0 && error == 0);
  }

  int getFd() { return sockfd; }

  bool send(const uint8_t *message, size_t size) {
    return sendto(
               sockfd, message, size, 0, (const struct sockaddr *)&serverAddr,
               sizeof(serverAddr)) != -1;
  }

  bool send(const std::vector<uint8_t> &message) {
    return send(message.data(), message.size());
  }

  ssize_t read(uint8_t *buffer, size_t size) {
    return ::read(sockfd, buffer, size);
  }

  ssize_t read(char *buffer, size_t size) {
    return ::read(sockfd, buffer, size);
  }

  void close() { ::close(sockfd); }

  int                sockfd;
  struct sockaddr_in serverAddr;
};

#endif