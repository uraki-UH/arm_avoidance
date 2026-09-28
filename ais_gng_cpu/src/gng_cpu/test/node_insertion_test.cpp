#include "cpu/cugng.hpp"
#include <fuzzrobo/libgng/api.h>
#include <iostream>
#include <stdexcept>

void require(bool is_valid, const char *message) {if (!is_valid) {throw std::runtime_error(message);}}

struct fixture {
    NodeConfig config{};
    EdgeConfig edges{};
    OtherConfig input{};
    CUGNG core;
    VoxelGrid grid;
    std::vector<Vec3f> points;
    std::vector<uint8_t> labels;
    fixture(int max_nodes = 64) {
        config.num_max = max_nodes;
        for (auto &dist : config.vigilance2) {dist = .0625f;}
        input.node_grid = input.voxel_grid_unit = 1;
        input.point_cloud_num = 100;
        input.x_min = input.y_min = input.z_min = -5;
        input.x_max = input.y_max = input.z_max = 5;
        require(core.init(&config, &edges, &input), "内部初期化");
        Vec3f first(0, 0, 0), second(0, .2f, 0);
        core.add_node(first); core.add_node(second); core.connect(0, 1);
        grid.init(&core.voxel_config, &input); labels.resize(100);
        core.enable_node_insertion = true;
        core.insertion_owners.assign(max_nodes, {UINT32_MAX, UINT32_MAX, UINT32_MAX});
        gng_insertion_plane plane;
        plane.normal[2] = plane.tangent_u[0] = plane.tangent_v[1] = 1;
        plane.min_u = plane.min_v = -2; plane.max_u = plane.max_v = 2;
        core.insertion_planes = {plane};
        core.insertion_owners[0] = {0, core.nodes[0].frame, 0};
    }
    void age_only() {
        core.insertion_stats = {};
        grid.applyFilter(points, points.size(), labels);
        core.age_unobserved_nodes(grid);
    }
    void run() {
        core.insertion_stats = {};
        grid.applyFilter(points, points.size(), labels);
        core.age_unobserved_nodes(grid);
        core.getDownSampling(grid.filtered_pcl, grid.filtered_pcl_num, labels, &grid, &points);
    }
};

int main() {
    for (const bool enable_insertion : {false, true}) {
        fixture test;
        test.core.enable_node_insertion = enable_insertion;
        test.core.nodes[0].age_s1 = test.core.nodes[1].age_s1 = 6;
        test.points = {{.2f,.1f,0}};
        test.run();
        require(test.core.nodes[0].age_s1 == (enable_insertion ? 0 : 6),
            "占有更新有効時だけ、入力を警戒領域内で受け持つ最近傍の寿命更新");
        require(test.core.nodes[1].age_s1 == 6,
            "同一占有セルでも最近傍でないノードの一括保護なし");
        test.core.nodes[0].age_s1 = test.core.nodes[1].age_s1 = 6;
        test.points = {{.6f,.1f,0}};
        test.run();
        require(test.core.nodes[0].age_s1 == 6 && test.core.nodes[1].age_s1 == 6,
            "最近傍でも既存警戒領域外の入力による寿命更新なし");
    }
    {
        fixture test; test.points = {{.6f,.1f,.02f},{.01f,0,0},{.02f,0,0}};
        test.run(); require(test.core.node_num == 2 && test.core.insertion_stats.num_plane_rejected_cells == 1,
            "疎でも有限平面で説明できる入力の除外");
        test.core.insertion_planes[0].max_u = .3;
        test.run(); require(test.core.insertion_stats.num_added_nodes == 1, "無限延長ではなく有限領域外への実測挿入");
        require(test.core.nodes[2].pos.squaredNorm(test.points[0]) == 0 && test.core.nodes[2].edge_num == 1 &&
            test.core.nodes[2].edges[0] == 0,
            "実測位置への追加と同時に最近傍1ノードだけへ接続");
        require(test.core.nodes[0].edge_num == 2 && test.core.nodes[1].edge_num == 1,
            "最近傍への双方向接続と第2近傍への追加接続なし");
        test.run(); require(test.core.insertion_stats.num_added_nodes == 0, "同じ観測の重複追加なし");
        require(test.core.nodes[2].edge_num > 0,
            "後続入力の最近傍対として選択された場合の通常接続");
    }
    {
        fixture test; test.core.enable_node_insertion = false;
        test.points = {{.6f,.1f,.2f}};
        test.run();
        require(test.core.node_num == 3 && test.core.nodes[2].edge_num == 0,
            "直接追加無効時の従来接続動作の維持");
    }
    {
        fixture test; test.points = {{.6f,.1f,.2f},{.01f,0,0},{.02f,0,0}};
        test.run(); require(test.core.insertion_stats.num_added_nodes == 1, "面外の未カバー観測への即時追加");
    }
    {
        fixture test; test.points = {{.6f,.1f,.02f},{.01f,0,0},{.02f,0,0}};
        test.core.insertion_owners[0].frame++;
        test.run(); require(test.core.insertion_stats.num_added_nodes == 1, "再利用IDへの古い平面所属の不適用");
    }
    {
        fixture test; test.points = {{.6f,.02f,.1f},{.01f,0,0},{.02f,0,0}};
        auto &plane = test.core.insertion_planes[0];
        plane.normal[2] = plane.tangent_v[1] = 0; plane.normal[1] = plane.tangent_v[2] = 1;
        test.run(); require(test.core.insertion_stats.num_plane_rejected_cells == 1, "回転した平面の説明判定");
    }
    {
        fixture test; test.points = {{.6f,.1f,.2f}};
        test.run(); require(test.core.node_num == 3 && test.core.insertion_stats.num_added_nodes == 1, "1点セルでも元点へ直接追加");
        test.points = {{.1f,.1f,.02f},{.11f,.1f,.02f},{.12f,.1f,.02f}};
        test.run(); require(test.core.insertion_stats.num_added_nodes == 0, "既存ノードでカバーされた占有の除外");
        test.points.clear(); test.run(); require(test.core.insertion_stats.num_added_nodes == 0, "空入力の追加なし");
    }
    {
        fixture test; test.points = {{3.1f,.1f,.2f},{3.2f,.1f,.2f},{3.4f,.1f,.2f}};
        test.run(); require(test.core.node_num == 3 && test.core.insertion_stats.num_added_nodes == 1,
            "離れた入力にも元点1点を追加");
        require(test.core.nodes[2].edge_num == 0,
            "既存探索範囲に候補がない場合の遠方への接続なし");
        test.core.check_delete_no_edge_and_decay_eta();
        require(test.core.node_num == 3, "観測のある孤立した移動先ノードの即時削除なし");
    }
    {
        fixture test(2); test.points = {{.6f,.1f,.2f},{.01f,0,0},{.02f,0,0}};
        test.run(); require(test.core.node_num == 2 && test.core.insertion_stats.num_capacity_rejected_cells == 1,
            "ノード上限での既存ノード削除なし");
    }
    {
        fixture test;
        test.points = {{.6f,.1f,.2f},{.01f,0,0},{.02f,0,0},
            {-.6f,.1f,.2f},{-.01f,0,0},{-.02f,0,0}};
        test.run(); require(test.core.insertion_stats.num_added_nodes == 2 && test.core.insertion_stats.num_checked_cells == 2,
            "固定補助枠なしで未カバーセルへ追加");
    }
    {
        fixture first, second; first.core.enable_node_insertion = second.core.enable_node_insertion = false;
        first.points = second.points = {{.6f,.1f,.2f},{.01f,0,0},{.02f,0,0}};
        second.core.insertion_config.max_unobserved_frames = 1;
        first.run(); second.run(); require(first.core.node_num == second.core.node_num, "OFF時の新設定による影響なし");
    }
    {
        fixture test; Vec3f anchor(0, .4f, 0); const auto id = test.core.add_node(anchor);
        test.core.connect(0, id); test.core.insertion_owners[0].id = UINT32_MAX;
        test.core.insertion_owners[id] = {id, test.core.nodes[id].frame, 0};
        test.points = {{.6f,.1f,.02f},{.01f,0,0},{.02f,0,0}};
        test.run(); require(test.core.insertion_stats.num_plane_rejected_cells == 1,
            "最近傍が平面未所属でも隣接ノードの所属平面で説明可能");
    }
    {
        fixture test;
        for (auto &dist : test.core.gng_config.vigilance2) {dist = .0225f;}
        test.points = {{.2f,0,.1f},{0,0,0}};
        test.run(); require(test.core.insertion_stats.num_added_nodes == 1,
            "2点セル・0.25 m以内でも既存警戒領域外なら補助挿入");
        test.run(); require(test.core.insertion_stats.num_added_nodes == 0, "補助挿入の重複なし");
    }
    {
        fixture test;
        test.points = {{.6f,.1f,.2f},{-.6f,.1f,.2f},{.1f,1.6f,.2f}};
        test.run(); require(test.core.node_num == 5 && test.core.insertion_stats.num_added_nodes == 3,
            "全未カバーセルの直接追加");
    }
    {
        fixture test; Vec3f point(1.2f,.1f,.3f);
        const auto id = test.core.add_node(point); test.core.connect(0, id);
        test.points = {{0,0,0}};
        test.age_only(); test.age_only();
        require(test.core.nodes[id].id == id, "未観測猶予内の保持");
        test.points.push_back(point); test.age_only();
        require(test.core.insertion_stats.num_aged_nodes == 0, "再観測による未観測寿命リセット");
        test.points.resize(1); test.age_only(); test.age_only();
        require(test.core.nodes[id].id == id, "再観測後の猶予を再計数");
        test.age_only();
        require(test.core.nodes[id].id == NODE_NOID && test.core.insertion_stats.num_removed_nodes == 1,
            "平面説明不能・連続3入力未観測の削除");
        const auto reused = test.core.add_node(point); test.core.connect(0, reused);
        require(reused == id, "削除IDの再利用");
        test.age_only(); test.age_only();
        require(test.core.nodes[id].id == id, "再利用IDに旧未観測寿命の持越しなし");
        test.core.enable_node_insertion = false; test.age_only();
        test.core.enable_node_insertion = true; test.age_only();
        require(test.core.nodes[id].id == id, "無効化中の寿命リセット");
    }
    {
        fixture test; Vec3f point(1.2f,.1f,.02f);
        const auto id = test.core.add_node(point); test.core.connect(0, id);
        test.points = {{0,0,0}};
        for (int iter = 0; iter < 5; ++iter) {test.age_only();}
        require(test.core.nodes[id].id == id && test.core.insertion_stats.num_aged_nodes == 0,
            "未観測でも隣接平面で説明可能なノードは早期削除対象外");
        ++test.core.insertion_owners[0].frame;
        for (int iter = 0; iter < 3; ++iter) {test.age_only();}
        require(test.core.nodes[id].id == NODE_NOID, "古い平面所属による誤保護なし");
    }
    {
        fixture test(3); Vec3f point(1.2f,.1f,.3f);
        const auto id = test.core.add_node(point); test.core.connect(0, id);
        test.core.insertion_config.max_unobserved_frames = 1;
        test.points = {{0,0,0},{2.2f,.1f,.3f}};
        test.run();
        require(test.core.node_num == 3 && test.core.insertion_stats.num_removed_nodes == 1 &&
            test.core.insertion_stats.num_added_nodes == 1 &&
            test.core.nodes[id].pos.squaredNorm(test.points[1]) == 0,
            "入力段階の未観測削除で空いた枠を同じ入力の移動先へ再利用");
    }
    {
        fixture test; Vec3f point(1.2f,.1f,.3f);
        const auto id = test.core.add_node(point); test.core.connect(0, id);
        test.points = {{0,0,0}};
        test.grid.enable_voxel_downsampling = false;
        for (int iter = 0; iter < 5; ++iter) {test.age_only();}
        require(test.core.nodes[id].id == id, "voxel無効時の早期削除なし");
    }
    // 公開入口の不正指定・一入力寿命。通常／製品の両ビルドで共用。
    gng_node_insertion_input value;
    const gng_plane_contact_voxel *contacts = nullptr;
    float contact_size = 0;
    gng_sampling_node_ref contact_ref{0, 0};
    require(gng_get_plane_contact_voxels(&contact_ref, 1, &contacts, &contact_size) == 0,
        "未初期化での接触セル取得の拒否");
    require(!gng_set_node_insertion(&value), "初期化前の拒否");
    gng_setParameter("node.num_max", 0, 64); gng_setParameter("node.learning_num", 0, 0);
    gng_setParameter("input.voxel_grid_unit", 0, 1); gng_setParameter("node.grid", 0, 1);
    require(gng_init() == SUCCESS, "公開API初期化");
    std::vector<Vec3> input{{.1f,.1f,.2f},{.2f,.1f,.2f},{.4f,.1f,.2f}};
    LiDAR_Config lidar; lidar.point_step = sizeof(Vec3);
    const auto submit = [&] {gng_setPointCloud(reinterpret_cast<const uint8_t *>(input.data()), input.size(), &lidar);};
    submit(); require(gng_set_node_insertion(&value), "空平面集合の登録"); gng_exec();
    require(gng_getTopologicalMap().node_num > 0, "学習0回でも同じ入力内の通常追加");
    gng_exec(); require(gng_get_node_insertion_stats().num_added_nodes == 0, "実行後の失効");
    submit(); require(gng_set_node_insertion(&value), "再登録"); submit(); gng_exec();
    require(gng_get_node_insertion_stats().num_checked_cells == 0, "入力置換後の失効");
    value.max_unobserved_frames = 0;
    require(!gng_set_node_insertion(&value), "未観測寿命0の拒否");
    value = {};
    for (const double invalid : {-1., double(NAN), double(INFINITY)}) {
        value.max_plane_dist_th = invalid; require(!gng_set_node_insertion(&value), "不正平面距離の拒否");
    }
    value = {}; gng_insertion_plane plane; value.planes = &plane; value.num_planes = 1;
    require(!gng_set_node_insertion(&value), "不正な平面基底の拒否");
    value = {}; value.num_owners = 1; require(!gng_set_node_insertion(&value), "null所属配列の拒否");
    value = {}; plane.normal[2] = plane.tangent_u[0] = plane.tangent_v[1] = 1;
    plane.min_u = plane.min_v = -2; plane.max_u = plane.max_v = 2;
    const auto map = gng_getTopologicalMap();
    gng_insertion_owner owner{map.nodes[0].id, map.nodes[0].frame, 0};
    plane.center[2] = .2;
    value.planes = &plane; value.num_planes = 1; value.owners = &owner; value.num_owners = 1;
    input = {{.7f,.1f,.2f},{0,.1f,.2f},{0,.1f,.2f}};
    submit(); require(gng_set_node_insertion(&value), "有限平面の登録");
    plane.center[2] = 100; owner.frame = UINT32_MAX;
    gng_exec(); require(gng_get_node_insertion_stats().num_plane_rejected_cells == 1,
        "登録後の呼出側配列変更に影響されない平面・所属コピー");
    require(gng_set_node_insertion(nullptr), "明示解除");
    const auto contact_map = gng_getTopologicalMap();
    contact_ref = {contact_map.nodes[0].id, contact_map.nodes[0].frame};
    require(gng_get_plane_contact_voxels(&contact_ref, 1, &contacts, &contact_size) == 1 &&
        contact_size == 1 && contacts[0].contact == 1 && contacts[0].num_points == 3,
        "既存占有セルと支持点数の再利用");
    require(contacts[0].center.x == .5f && contacts[0].center.y == .5f && contacts[0].center.z == .5f,
        "点群重心ではなくグリッド原点基準のセル中心");
    ++contact_ref.frame;
    require(gng_get_plane_contact_voxels(&contact_ref, 1, &contacts, &contact_size) == 0, "古い世代の除外");
    --contact_ref.frame;
    input = {{.7f,.1f,.2f},{1.1f,.1f,.2f},{1.1f,1.1f,.2f},{-.1f,.1f,.2f}};
    submit();
    require(gng_get_plane_contact_voxels(&contact_ref, 1, &contacts, &contact_size) == 0,
        "入力置換から実行までの古いセル取得の拒否");
    gng_exec();
    require(gng_get_plane_contact_voxels(&contact_ref, 1, &contacts, &contact_size) == 3,
        "同一・正負の面共有隣接のみ、斜めセルの除外");
    uint32_t num_same = 0, num_adjacent = 0;
    for (uint32_t idx = 0; idx < 3; ++idx) {
        num_same += contacts[idx].contact == 1; num_adjacent += contacts[idx].contact == 2;
    }
    require(num_same == 1 && num_adjacent == 2, "同一セルと隣接セルの色分け用区分");
    require(gng_get_plane_contact_voxels(nullptr, 0, &contacts, &contact_size) == 0 && contacts == nullptr,
        "平面なしの表示消去用空集合");
    input.clear(); submit(); gng_exec();
    require(gng_get_plane_contact_voxels(&contact_ref, 1, &contacts, &contact_size) == 0, "空入力の空集合");
    // 公開実行経路でも、互いに接続のない観測セルの元点を保持。
    input = {{5.1f,0,.3f},{10.1f,0,.3f},{15.1f,0,.3f}};
    value = {};
    submit(); require(gng_set_node_insertion(&value), "占有更新の登録"); gng_exec();
    const auto has_point = [&](float x) {
        const auto current = gng_getTopologicalMap();
        for (uint32_t idx = 0; idx < current.node_num; ++idx) {
            if (current.nodes[idx].pos.x == x && current.nodes[idx].pos.z == .3f) {return true;}
        }
        return false;
    };
    require(has_point(5.1f) && has_point(10.1f) && has_point(15.1f),
        "学習0回・全フレーム処理後も観測された孤立元点の保持");
    input.resize(2);
    for (int iter = 0; iter < 3; ++iter) {
        submit(); require(gng_set_node_insertion(&value), "連続入力の登録"); gng_exec();
    }
    require(has_point(5.1f) && has_point(10.1f) && !has_point(15.1f),
        "公開実行経路の再観測保持と連続未観測削除");
    std::cout << "node_insertion_test=passed\n";
}
