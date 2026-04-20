#pragma once

#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

// UDP送信クラス
class UdpSender {
public:
  UdpSender(const std::string &ip, int port) {
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
      perror("Socket creation failed");
      exit(EXIT_FAILURE);
    }

    memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &server_addr_.sin_addr);
  }

  ~UdpSender() { close(sock_); }

  void send(const std::string &message) {
    sendto(sock_, message.c_str(), message.size(), 0,
           (struct sockaddr *)&server_addr_, sizeof(server_addr_));
  }

private:
  int sock_;
  struct sockaddr_in server_addr_;
};

// UDP受信クラス
class UdpReceiver {
public:
  UdpReceiver(int port) {
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
      perror("Socket creation failed");
      exit(EXIT_FAILURE);
    }

    memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port);
    server_addr_.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock_, (struct sockaddr *)&server_addr_, sizeof(server_addr_)) <
        0) {
      perror("Bind failed");
      exit(EXIT_FAILURE);
    }
  }

  ~UdpReceiver() { close(sock_); }

  std::string receive() {
    char buffer[1024];
    socklen_t len = sizeof(client_addr_);
    int n = recvfrom(sock_, buffer, sizeof(buffer) - 1, 0,
                     (struct sockaddr *)&client_addr_, &len);
    if (n > 0) {
      buffer[n] = '\0';
      return std::string(buffer);
    }
    return "";
  }

private:
  int sock_;
  struct sockaddr_in server_addr_, client_addr_;
};
