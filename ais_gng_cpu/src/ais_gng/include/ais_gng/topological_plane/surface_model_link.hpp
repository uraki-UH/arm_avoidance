#pragma once

#include <cstdint>
#include <stdexcept>

namespace fuzzrobo::surface_model {

// 順序付き端点キーと接続状態の内部表現。保存・通信への直接利用なし
class smooth_link {
public:
  explicit smooth_link(std::uint64_t key = 0, bool is_connected = false)
      : value_(0) {
    if ((key >> 63) != 0) {throw std::out_of_range("surface link key exceeds 63 bits");}
    value_ = (key << 1) | static_cast<std::uint64_t>(is_connected);
  }
  std::uint64_t key() const {return value_ >> 1;}
  bool is_connected() const {return (value_ & 1U) != 0;}
  void set_connected(bool is_connected) {value_ = (value_ & ~std::uint64_t{1}) | is_connected;}
  bool operator<(const smooth_link &other) const {return value_ < other.value_;}
  bool operator==(const smooth_link &other) const {return value_ == other.value_;}
private:
  std::uint64_t value_;
};

}  // namespace fuzzrobo::surface_model
