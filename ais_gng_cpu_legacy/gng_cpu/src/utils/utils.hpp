#pragma once

#include <fuzzrobo/libgng/api.h>

// 外のライブラリ
#include <stdarg.h>

#include <algorithm>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <random>
#include <set>
#include <string>
#include <vector>
#include <unordered_set>
#include <cstring>
#include <cassert>
#include <cfloat>
#include <map>

#include "defaults.hpp"

// 公開鍵
#if (GNG_VERSION == 0)
    #define VERSION_MOVE // 移動型パーセプション
    #include "../auth/pub_key/pub_key_move.hpp"
#elif (GNG_VERSION == 1)
    #define VERSION_STATIC // 静的型パーセプション
    #include "../auth/pub_key/pub_key_static.hpp"
#elif (GNG_VERSION == 2)
    #define VERSION_MAP // マッピングオンリー
    #include "../auth/pub_key/pub_key_map.hpp"
#endif

// 名前空間
using namespace std;

// マクロ
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define LIMIT(a, b, c) ((a) < (b) ? (b) : ((a) > (c) ? (c) : (a)))
#define millis(a) (std::chrono::duration_cast<std::chrono::milliseconds>(a).count())

// 関数
float squaredNormXY(Vec3 &p1, Vec3 &p2);
vector<int> make_rand_array(const int size, int rand_min, int rand_max);
vector<int> make_rand_array_unique(const int size, int rand_min, int rand_max);

struct Time{
    float lpf_dt_T;
    float lpf_a;
    chrono::system_clock::time_point ts_prev;
    void update(float time_constant) {
        auto ts_now = chrono::system_clock::now();
        float dt  = (float)chrono::duration_cast<chrono::milliseconds>(ts_now - ts_prev).count() / 1000.f;
        ts_prev = ts_now;
        // log.println("%f", dt);
        dt = LIMIT(dt, 0.1, 0.5);
        lpf_dt_T = 1.f / (dt + time_constant);
        lpf_a = time_constant * lpf_dt_T;
    }
};