#pragma once

#include <concepts>
#include <cstdint>
#include <stdexcept>
#include <type_traits>

// 狭幅変換前の値域検査と、整数幅に依存しない値域の保持
template<std::uint64_t max_value, std::uint64_t min_value = 0>
class bounded_uint {
  static_assert(min_value <= max_value);
  using storage_type = std::conditional_t<(max_value <= UINT8_MAX), std::uint8_t,
      std::conditional_t<(max_value <= UINT16_MAX), std::uint16_t,
      std::conditional_t<(max_value <= UINT32_MAX), std::uint32_t, std::uint64_t>>>;
public:
  constexpr bounded_uint() : value_(min_value) {}
  template<std::integral integer_type>
  explicit constexpr bounded_uint(integer_type value) {
    if constexpr (std::is_signed_v<integer_type>) {
      if (value < 0) {throw std::out_of_range("bounded integer out of range");}
    }
    const auto checked_value = static_cast<std::uint64_t>(value);
    if (checked_value < min_value || checked_value > max_value) {
      throw std::out_of_range("bounded integer out of range");
    }
    value_ = static_cast<storage_type>(checked_value);
  }
  constexpr storage_type value() const {return value_;}
private:
  storage_type value_;
};
