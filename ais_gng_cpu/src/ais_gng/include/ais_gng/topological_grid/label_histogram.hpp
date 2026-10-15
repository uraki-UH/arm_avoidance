#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace fuzzrobo::topological_grid::detail
{

// uint8ラベルの存在ビットと、存在順の頻度配列。頻度値のsize_t精度の保持
class label_histogram
{
public:
  void increment(std::uint8_t label)
  {
    const std::size_t word_idx = label / 64;
    const std::uint64_t bit = std::uint64_t{1} << (label % 64);
    const std::size_t idx = count_idx(word_idx, bit);
    if ((label_words_[word_idx] & bit) != 0) {
      ++counts_[idx];
    } else {
      counts_.insert(counts_.begin() + idx, 1);
      label_words_[word_idx] |= bit;
    }
  }

  void decrement(std::uint8_t label)
  {
    const std::size_t word_idx = label / 64;
    const std::uint64_t bit = std::uint64_t{1} << (label % 64);
    if ((label_words_[word_idx] & bit) == 0) {
      throw std::logic_error("label histogram count is absent");
    }
    const std::size_t idx = count_idx(word_idx, bit);
    if (--counts_[idx] == 0) {
      counts_.erase(counts_.begin() + idx);
      label_words_[word_idx] &= ~bit;
    }
  }

  std::uint8_t dominant_label() const noexcept
  {
    std::uint8_t dominant = 0;
    std::size_t dominant_count = 0, idx = 0;
    for (std::size_t word_idx = 0; word_idx < label_words_.size(); ++word_idx) {
      auto word = label_words_[word_idx];
      while (word != 0) {
        // ラベル昇順の走査。同数の場合の小さいラベルの優先
        if (counts_[idx] > dominant_count) {
          dominant_count = counts_[idx];
          dominant = static_cast<std::uint8_t>(word_idx * 64 + __builtin_ctzll(word));
        }
        ++idx;
        word &= word - 1;
      }
    }
    return dominant;
  }

private:
  std::size_t count_idx(std::size_t word_idx, std::uint64_t bit) const noexcept
  {
    std::size_t idx = 0;
    for (std::size_t preceding = 0; preceding < word_idx; ++preceding) {
      idx += __builtin_popcountll(label_words_[preceding]);
    }
    return idx + __builtin_popcountll(label_words_[word_idx] & (bit - 1));
  }

  std::array<std::uint64_t, 4> label_words_{};
  std::vector<std::size_t> counts_;
};

}  // 名前空間fuzzrobo::topological_grid::detailの終端
