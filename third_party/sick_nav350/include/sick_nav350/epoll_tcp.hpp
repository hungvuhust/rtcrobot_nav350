#ifndef _EPOLL_UDP_HPP_
#define _EPOLL_UDP_HPP_

#include <iostream>
#include <sys/epoll.h>
#include <unistd.h>
#include <vector>

class Epoll {
public:
  Epoll() {
    epoll_fd= epoll_create1(0);
    if (epoll_fd == -1) {
      throw std::runtime_error("Error creating epoll instance");
    }
  }

  ~Epoll() { close(); }

  void addSocket(int sock_fd, uint32_t events) {
    struct epoll_event ev;
    ev.events = events;
    ev.data.fd= sock_fd;
    if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, sock_fd, &ev) == -1) {
      throw std::runtime_error("Error adding socket to epoll");
    }
  }

  std::vector<struct epoll_event> wait(int timeout= -1) {
    std::vector<struct epoll_event> events(10);
    int nfds= epoll_wait(epoll_fd, events.data(), events.size(), timeout);
    if (nfds == -1) {
      throw std::runtime_error("Error during epoll wait");
    }
    events.resize(nfds);
    return events;
  }

  ssize_t read(int fd, void *buf, size_t count) {
    return ::read(fd, buf, count);
  }
  int getEpollFd() { return epoll_fd; }

  void close() { ::close(epoll_fd); }

private:
  int epoll_fd;
};

#endif // _EPOLL_UDP_HPP_
