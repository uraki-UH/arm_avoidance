# Offline Guide

このドキュメントは、GNG / VLUT の学習・再構築・オフライン確認をまとめたものです。

## 1. オフライン学習

`offline_urdf_trainer` を使って、URDF から GNG と VLUT を生成します。

```bash
ros2 run gng_safety offline_urdf_trainer --id my_robot_v1 --res 0.02
```

VLUT だけ再構築する場合:

```bash
ros2 run gng_safety offline_urdf_trainer --id my_robot_v1 --vlut-only --res 0.02
```

## 2. VLUT 再構築

```bash
cd ~/uraki_ws/gng_vlut_system
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gng_safety --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_ODE=OFF -DUSE_DRAWSTUFF=OFF
source install/setup.bash
```

例:

```bash
ros2 run gng_safety offline_urdf_trainer --id topoarm_real_v1 --vlut-only --res 0.02
```

## 3. 出力

- `<id>_phase2.bin`
- `gng_spatial_correlation.bin`
- `config.txt`

## 4. オフラインでの確認

旧来の単体シミュレータは legacy として残していますが、ROS 2 側は `gng_safety` + RViz2 を使うのが基本です。

## 5. 可視化スクリプト

`scripts/README_visualization.md` にあった関節角度 / 可操作性プロットは、今後はこのガイドの補足として扱います。

### 関節角度

```bash
python3 scripts/visualize_joint_angles.py
```

### 可操作性

```bash
python3 scripts/visualize_manipulability.py --file build/manipulability_data.csv
```
