#pragma once

#include <fuzzrobo/libgng/api.h>
#include <fuzzrobo/libgng/observation_angle_range.hpp>
#include <fuzzrobo/libgng/observation_pixel_view.hpp>

#ifdef __cplusplus
extern "C" {
#endif

// 一学習入力の観測情報。原点と整数角度表はGNG点群と同じ座標系。
struct gng_observation_input {
    Vec3 origin{};
    uint8_t has_origin = 0;
    gng_observation::pixel_view pixels{};
    const gng_observation::ray_angles *angle_table = nullptr;
    uint32_t table_num = 0;
};
// gng_setPointCloud後、gng_exec前の一括置換。nullptrは入力観測情報の解除。
// 構造体は値コピー、画素・間引き番号・角度表の借用メモリはgng_exec完了まで有効なもの。
// 返値1は受付成功、0は無効入力または機能OFF。非有限原点は観測全体の無効化。
// pixels未指定はレイ計算。無効ビュー・表は返値0かつ有効原点によるレイ計算への復帰。
uint8_t gng_set_observation_input(const gng_observation_input *input);

// 一学習フレームの第一勝者レイ範囲。値返却による次回更新からの独立。
gng_observation::angle_range gng_get_observation_angle_range(uint16_t node_id);

// 直近の学習出力に対応する原点と参照件数。次フレーム用の入力指定とは独立。
struct gng_observation_frame {
    Vec3 origin{};
    uint32_t frame_number = 0;
    uint8_t has_origin = 0;
    uint32_t pixel_hit_num = 0;
    uint32_t ray_num = 0;
};
gng_observation_frame gng_get_observation_frame();

#ifdef __cplusplus
}
#endif
