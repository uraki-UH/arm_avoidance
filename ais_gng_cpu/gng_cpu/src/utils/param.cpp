#include "param.hpp"

bool GridConfig::init(GridConfig _c){
    if(_c.x_max < _c.x_min)
    return false;
    if (_c.y_max < _c.y_min)
        return false;
    if (_c.z_max < _c.z_min)
        return false;
    _c.unit_1 = 1.f / _c.unit;
    _c.max[0] = (uint32_t)((_c.x_max - _c.x_min) * _c.unit_1);
    _c.max[1] = (uint32_t)((_c.y_max - _c.y_min) * _c.unit_1);
    _c.max[2] = (uint32_t)((_c.z_max - _c.z_min) * _c.unit_1);
    uint64_t max64 = (uint64_t)_c.max[0]* (uint64_t)_c.max[1] * (uint64_t)_c.max[2];
    if(max64 >= UINT32_MAX)
        return false;
    // 初期化
    x_max = _c.x_max;
    x_min = _c.x_min;
    y_max = _c.y_max;
    y_min = _c.y_min;
    z_max = _c.z_max;
    z_min = _c.z_min;
    unit = _c.unit;
    unit_1 = _c.unit_1;
    max[0] = _c.max[0];
    max[1] = _c.max[1];
    max[2] = _c.max[2];
    maxXY = _c.max[0] * _c.max[1];
    maxXYZ = _c.max[0] * _c.max[1] * _c.max[2];
    return true;
}

bool GridConfig::isRange(Vec3f &p){
    if (p[0] < x_min || p[0] > x_max) return false;
    if (p[1] < y_min || p[1] > y_max) return false;
    if (p[2] < z_min || p[2] > z_max) return false;
    return true;
}

bool GridConfig::isRange(float *pos){
    if (pos[0] < x_min || pos[0] > x_max) return false;
    if (pos[1] < y_min || pos[1] > y_max) return false;
    if (pos[2] < z_min || pos[2] > z_max) return false;
    return true;
}

uint32_t GridConfig::getIndex(int i, int j, int k) {
    return i + (uint32_t)j * max[0] + (uint32_t)k * maxXY;
}

uint32_t GridConfig::getIndex(Vec3f &p){
    if(!isRange(p))
        return UINT32_MAX; // 範囲外
    int i = (int)((p[0] - x_min) * unit_1);
    int j = (int)((p[1] - y_min) * unit_1);
    int k = (int)((p[2] - z_min) * unit_1);
    return getIndex(i, j, k);
}

uint32_t GridConfig::getIndex(float *p){
    if(!isRange(p))
        return UINT32_MAX; // 範囲外
    int i = (int)((p[0] - x_min) * unit_1);
    int j = (int)((p[1] - y_min) * unit_1);
    int k = (int)((p[2] - z_min) * unit_1);
    return getIndex(i, j, k);
}

Param::Param() {
    init();
}
Param::~Param() {
}

void Param::init() {
    config.point_cloud_num = 384000; // JT16:4800, AT128: 384000
    config.node_grid = 0.5; // ノードのグリッドサイズ
    config.voxel_grid_unit = 0.1; // ボクセルグリッドのサイズ
    config.local_coordinates = false; // ローカル座標系を使用するか
    config.x_min = DEFAULT_X_MIN;
    config.x_max = DEFAULT_X_MAX;
    config.y_min = DEFAULT_Y_MIN;
    config.y_max = DEFAULT_Y_MAX;
    config.z_min = DEFAULT_Z_MIN;
    config.z_max = DEFAULT_Z_MAX;

    node.num_max = 2000;
    node.learning_num = 4000;
    node.unknown_learning_rate = 5;
    node.eta_s1 = 0.08;
    node.eta_s1_2 = 1.f - node.eta_s1;
    node.eta_s2 = 0.008;
    node.eta_s2_2 = 1.f - node.eta_s2;
    node.edge_age_max = 100;
    // node.dynamic_vel_min = 0.5;
    // node.dynamic_lpf_time = 0.5;
    node.s1_reset_range2 = 0.1 * 0.1; // ノードの選択回数リセット範囲
    node.ds_range_max2 = 0.2 * 0.2; // ダウンサンプリングの範囲
    node.static_age_min = -1;

    node.vigilance2[0] = 0.1 * 0.1; // default
    node.vigilance2[1] = 0.3 * 0.3; // safe
    node.vigilance2[2] = 0.5 * 0.5; // wall
    node.vigilance2[3] = 0.05 * 0.05; // unknown
    node.s1_age[0] = 6; // default
    node.s1_age[1] = 6; // safe
    node.s1_age[2] = 6;  // wall
    node.s1_age[3] = 3; // unknown
    node.clusted_s1_age[0] = 20; // default
    node.clusted_s1_age[1] = 20;  // safe (clusted)
    node.clusted_s1_age[2] = 6;// wall (clusted)
    node.clusted_s1_age[3] = 3; // unknown (clusted)
    node.edge_distance_min2[0] = 0 * 0; // default
    node.edge_distance_min2[1] = 0.2 * 0.2;  // safe (clusted)
    node.edge_distance_min2[2] = 0.2 * 0.2;// wall (clusted)
    node.edge_distance_min2[3] = 0 * 0; // unknown (clusted)

    label.fuzz_unknown_1 = 1.f / 0.6;
    label.fuzz_min = 0.5;
    label.lpf_time = 0.5;

    cluster.num_min = 10;
    cluster.lpf_time = 0.5; // ラベリングのLPF時定数
    cluster.constant_age_ave_min = 30;
    cluster.plane_volume = 10.0;
    cluster.unknown_edge_distance_max2 = 0.3 * 0.3;
    cluster.other_edge_distance_max2 = 0.5 * 0.5;
    // cluster.human_dbscan_epsilon = 0.2;
    // cluster.human_dbscan_points_num_min = 10;
    // cluster.classify_car = false;
    cluster.human_radius = 0.6;
    cluster.human_hysteresis_age = 5;
    cluster.human_confirmation_age = 2;
    cluster.car_hysteresis_age = 5;
    cluster.car_confirmation_age = 2;

    // cluster.wall_normal_dot_max = 0.9;
    // cluster.human_age = 200;
    // cluster.human_division = true;
}

bool Param::setParameter(const char *p_c, const uint32_t index, const float value) {
    string p(p_c);

    // debug
    // if (p == "node.grid" && value > 0)
    //     config.node_grid = value;
    if (p == "node.num_max" && value >= 0)
        node.num_max = value;
    else if (p == "node.learning_num" && value >= 0)
        node.learning_num = value;
    else if (p == "node.unknown_learning_rate" && value >= 0.5 && value < 1)
        node.unknown_learning_rate = (int)(1.f / (1.f - (std::floor(value * 10) / 10.f)));
    else if (p == "node.eta_s1" && value >= 0 && value <= 1) {
        node.eta_s1 = value;
        node.eta_s1_2 = 1.f - node.eta_s1;
    }else if (p == "node.eta_s2" && value >= 0 && value <= 1){
        node.eta_s2 = value;
        node.eta_s2_2 = 1.f - node.eta_s2;
    }else if (p == "node.edge_age_max" && value >= 0)
        node.edge_age_max = value;
    else if (p == "node.s1_reset_range" && value > 0)
        node.s1_reset_range2 = value * value;
    else if (p == "node.static.age_min")
        node.static_age_min = value;
    else if (p == "ds.range_max" && value > 0)
        node.ds_range_max2 = value * value;
    else if (p == "label.fuzzy.unknown" && value >= 0) label.fuzz_unknown_1 =
        1.f / value;
    else if (p == "label.fuzzy.min" && value >= 0)
        label.fuzz_min = value;
    else if (p == "label.fuzzy.lpf_time_constant" && value >= 0)
        label.lpf_time = value;
    else if (p == "cluster.num_min" && value >= 1)
        cluster.num_min = value;
    else if (p == "cluster.velocity.lpf_time_constant" && value >= 0)
        cluster.lpf_time = value;
    else if (p == "cluster.constant.age_ave_min" && value >= 0)
        cluster.constant_age_ave_min = value;
    else if (p == "cluster.plane.volume" && value >= 0)
        cluster.plane_volume = value;
    else if (p == "cluster.unknown.edge_distance_max" && value >= 0)
        cluster.unknown_edge_distance_max2 = value * value;
    else if (p == "cluster.other.edge_distance_max" && value >= 0)
        cluster.other_edge_distance_max2 = value * value;
    // else if (p == "cluster.human.dbscan.epsilon" && value > 0)
    //     cluster.human_dbscan_epsilon = value;
    // else if (p == "cluster.human.dbscan.points_num_min" && value > 0)
    //     cluster.human_dbscan_points_num_min = value;
    // else if (p == "classify.car" && value >= 0)
    //     cluster.classify_car = (value == 1.f);
    else if (p == "cluster.human.radius" && value >= 0)
        cluster.human_radius = value;
    else if (p == "cluster.human.hysteresis_age" && value >= 0)
        cluster.human_hysteresis_age = value + 1;
    else if (p == "cluster.human.confirmation_age" && value >= 0)
        cluster.human_confirmation_age = value;
    else if (p == "cluster.car.hysteresis_age" && value >= 0)
        cluster.car_hysteresis_age = value + 1;
    else if (p == "cluster.car.confirmation_age" && value >= 0)
        cluster.car_confirmation_age = value;
    // else if (p == "cluster.wall.normal_angle_max" && value >= 0 && value <= 180)
    //     cluster.wall_normal_dot_max = cosf(value / 360 * 2 * M_PI);
    // else if (p == "cluster.human_age" && value >= 0)
    //     cluster.human_age = value;
    // else if (p == "cluster.human_division" && value >= 0)
    //     cluster.human_division = (value == 1.f);
    else if (p == "input.point_cloud_num" && value > 0)
        config.point_cloud_num = value;
    else if (p == "input.voxel_grid_unit" && value > 0)
        config.voxel_grid_unit = value;
#if defined(VERSION_MAP) || defined(VERSION_STATIC)
    else if (p == "input.x_min" && value >= DEFAULT_X_MIN && value <= DEFAULT_X_MAX)
        config.x_min = value;
    else if (p == "input.x_max" && value >= DEFAULT_X_MIN && value <= DEFAULT_X_MAX)
        config.x_max = value;
    else if (p == "input.y_min" && value >= DEFAULT_Y_MIN && value <= DEFAULT_Y_MAX)
        config.y_min = value;
    else if (p == "input.y_max" && value >= DEFAULT_Y_MIN && value <= DEFAULT_Y_MAX)
        config.y_max = value;
    else if (p == "input.z_min" && value >= DEFAULT_Z_MIN && value <= DEFAULT_Z_MAX)
        config.z_min = value;
    else if (p == "input.z_max" && value >= DEFAULT_Z_MIN && value <= DEFAULT_Z_MAX)
        config.z_max = value;
#else
    else if (p == "input.x_min")
        config.x_min = value;
    else if (p == "input.x_max")
        config.x_max = value;
    else if (p == "input.y_min")
        config.y_min = value;
    else if (p == "input.y_max")
        config.y_max = value;
    else if (p == "input.z_min")
        config.z_min = value;
    else if (p == "input.z_max")
        config.z_max = value;
    // ローカルで運用すれば，移動ロボットでも使えてしまうため
    else if (p == "input.local_coordinates" && value >= 0)
        config.local_coordinates = (value == 1.f);
#endif
    else if (p == "node.s1_age" && index < 4 && value > 0)
        node.s1_age[index] = value;
    else if (p == "node.clusted_s1_age" && index < 4 && value > 0)
        node.clusted_s1_age[index] = value;
    else if (p == "node.interval" && index < 4 && value >= 0){
        node.vigilance2[index] = (value) * (value);
        float edge_distance_min = MAX(0.f, value - 0.2f);
        node.edge_distance_min2[index] = edge_distance_min * edge_distance_min;
    }else{
        return false;
    }
    return true;
}