#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#ifndef enable_voxel_framework_build
#define enable_voxel_framework_build 0
#endif
#ifndef enable_voxel_fuzzy_build
#define enable_voxel_fuzzy_build 0
#endif
#ifndef enable_voxel_history_build
#define enable_voxel_history_build 0
#endif

namespace fuzzrobo::voxel_framework {

// 属性集計・ファジィ評価・履歴の静的構成。機能なしの型には保存領域なし。
template<bool enable_attributes_value, bool enable_fuzzy_value = false, bool enable_history_value = false>
struct features {
    static_assert(enable_attributes_value || (!enable_fuzzy_value && !enable_history_value), "ファジィ評価・履歴には属性集計が必要");
    static constexpr bool enable_attributes = enable_attributes_value;
    static constexpr bool enable_fuzzy = enable_fuzzy_value;
    static constexpr bool enable_history = enable_history_value;
};

using build_features = features<enable_voxel_framework_build != 0,
    enable_voxel_fuzzy_build != 0, enable_voxel_history_build != 0>;

// 履歴を使わない条件は履歴対応ビルドでも無状態。必要機能だけの明示選択。
template<bool enable_fuzzy_value = false, bool enable_history_value = false>
struct configured_features : features<build_features::enable_attributes,
    build_features::enable_fuzzy && enable_fuzzy_value, build_features::enable_history && enable_history_value> {
    static_assert(!enable_fuzzy_value || build_features::enable_fuzzy, "ファジィ評価がビルドに含まれていない構成");
    static_assert(!enable_history_value || build_features::enable_history, "履歴がビルドに含まれていない構成");
};

template<bool enable_history, class policy>
struct history_storage {};

template<class policy>
struct history_storage<true, policy> {
    struct record {
        typename policy::attributes_type attributes;
        double stamp_sec;
        uint64_t frame;
    };
    std::unordered_map<typename policy::key_type, record, typename policy::key_hash> records;
    uint64_t epoch = 0, frame = 0;
    double stamp_sec = 0;
    std::size_t max_cells = 0;
    bool has_frame = false;
};

template<class feature_set, class policy, bool enable_attributes = feature_set::enable_attributes>
class pipeline;

// 無拡張構成。属性型・集計関数・評価関数・履歴型の実体化なし。
template<class feature_set, class policy>
class pipeline<feature_set, policy, false> {
public:
    void begin_frame(uint64_t, double, std::size_t) const noexcept {}
    void end_frame() const noexcept {}
    void reset() const noexcept {}
    constexpr std::size_t num_history_cells() const noexcept {return 0;}
    template<class cell>
    decltype(auto) evaluate(const cell &input, policy &rules) const {
        return rules.baseline(input);
    }
};

// 集計はセル当たり一度。属性・更新規則・評価結果の型は利用側ポリシーの定義。
template<class feature_set, class policy>
class pipeline<feature_set, policy, true> : private history_storage<feature_set::enable_history, policy> {
public:
    void reset() noexcept {
        if constexpr (feature_set::enable_history) {
            this->records.clear(); this->has_frame = false; this->frame = 0;
        }
    }

    void begin_frame(uint64_t epoch, double stamp_sec, std::size_t max_cells) {
        if constexpr (feature_set::enable_history) {
            if (!std::isfinite(stamp_sec) || max_cells == 0) {
                reset(); throw std::invalid_argument("履歴の時刻・セル数上限の不正値");
            }
            if (!this->has_frame || epoch != this->epoch || stamp_sec <= this->stamp_sec ||
                max_cells != this->max_cells) {reset();}
            this->epoch = epoch; this->stamp_sec = stamp_sec; this->max_cells = max_cells;
            this->has_frame = true; ++this->frame;
        }
    }

    // 未観測セルの占有否定なし。今回扱わなかったセルの履歴だけを破棄。
    void end_frame() {
        if constexpr (feature_set::enable_history) {
            for (auto it = this->records.begin(); it != this->records.end();) {
                if (it->second.frame != this->frame) {it = this->records.erase(it);}
                else {++it;}
            }
        }
    }

    std::size_t num_history_cells() const noexcept {
        if constexpr (feature_set::enable_history) {return this->records.size();}
        else {return 0;}
    }

    template<class cell>
    auto evaluate(const cell &input, policy &rules) {
        try {
            auto attributes = rules.collect(input);
            if constexpr (feature_set::enable_history) {
                if (!this->has_frame) {throw std::logic_error("履歴フレームの開始前");}
                const auto key = rules.key(input);
                auto found = this->records.find(key);
                if (found != this->records.end() && found->second.frame == this->frame) {
                    throw std::logic_error("同一フレーム内のセルキー重複");
                }
                const auto *previous = found == this->records.end() ? nullptr : &found->second.attributes;
                const double elapsed_sec = previous ? this->stamp_sec - found->second.stamp_sec : 0;
                rules.update(attributes, previous, elapsed_sec);
                if (found == this->records.end()) {
                    // 上限到達時の全履歴走査・暗黙の上限拡張なし。
                    if (this->records.size() == this->max_cells) {
                        throw std::length_error("履歴セル数の上限超過");
                    }
                    this->records.emplace(key, typename history_storage<true, policy>::record{
                        attributes, this->stamp_sec, this->frame});
                } else {found->second = {attributes, this->stamp_sec, this->frame};}
            }
            if constexpr (feature_set::enable_fuzzy) {return rules.fuzzy(attributes);}
            else {return rules.evaluate(attributes);}
        } catch (...) {
            // 失敗フレームの部分履歴を次回へ持ち越さないための失効。
            reset(); throw;
        }
    }
};
}  // 名前空間fuzzrobo::voxel_framework
