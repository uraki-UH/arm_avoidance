#pragma once
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// データ送信用フォーマット生成
// "Normal Mode" format: 0,0,0,...,0, (comma separated values, confirmed
// trailing comma)
std::string generateSendData(const std::vector<int> &angles) {
  std::ostringstream oss;
  for (size_t i = 0; i < angles.size(); ++i) {
    oss << angles[i] << ",";
  }
  return oss.str();
}

// データ受信の解析 (Debug用)
void parseReceivedData(const std::string &data) {
  // Ideally this should parse into a struct, but for now we just print
  // The received data format is likely "vsd,interval:...,id,..."
  // We just print it to check connection.
  // std::cout << "[RX Parsing] " << data << std::endl;
}
