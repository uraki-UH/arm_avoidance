# offline_urdf_trainer 利用ガイド

`offline_urdf_trainer` は、URDFモデルから GNG（グラフ）と VLUT（空間ルックアップテーブル）を一貫して生成するためのツールです。

## 1. 実行方法

### 標準的な実行 (フル学習)
```bash
./build/offline_urdf_trainer --id my_robot_v1 --res 0.02
```

### VLUTのみの再構築 (GNG学習済みの場合)
```bash
./build/offline_urdf_trainer --id my_robot_v1 --vlut-only --res 0.02
```

## 2. パラメータ設定 (`config.txt`)

主要な設定項目：
*   `spatial_map_resolution`: VLUTの細かさ（メートル）。
*   `max_iterations`: GNGの学習回数（100万回〜）。
*   `max_node_num`: 生成する最大ノード数。
*   `spatial_map_min/max`: ロボットが動く空間の立方体範囲。

## 3. 入出力

### 入力
*   `urdf/*.urdf` または `*.xacro`: ロボットの形状・関節定義。
*   `meshes/*.stl`: 衝突判定用の3Dメッシュ。

### 出力 (場所: `gng_results/<id>/`)
1.  **`<id>_phase2.bin`**: 学習されたGNGネットワーク。
2.  **`gng_spatial_correlation.bin`**: 
    *   最新の自己構成形式（Version 2）。
    *   解像度とワークスペース範囲がファイル内に記録されており、オンライン実行時に自動認識されます。
3.  **`config.txt`**: 実行時の設定ファイルのコピー。

## 4. 特徴：自己構成機能 (Auto-Configuration)

今回のアップデートにより、出力された `gng_spatial_correlation.bin` は、**どの解像度・どの範囲で構築されたかを自分自身で知っています。** 

そのため、利用側（`online_integration` や将来の ROS 2 ノード）では、設定ファイルを意識することなくファイルをロードするだけで、正しい空間検索が即座に可能になります。
