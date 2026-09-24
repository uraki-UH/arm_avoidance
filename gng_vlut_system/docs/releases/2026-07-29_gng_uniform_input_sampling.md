# 2026-07-29 - GNG uniform input sampling

## 1. 要約

`input.point_cloud_num` を超える点群について、先頭点だけではなく入力全域から決定論的に等間隔抽出できるようにする。

- `graspnet.yaml` の入力点選択を `uniform` に変更する。
- 抽出後も GNG ノードの入力点 ID を元点群のインデックスとして publish する。

- `input.sampling_mode` パラメータに `head` と `uniform` を追加する。

- organized point cloud が上限を超えた際、画像上部の点だけが GNG 学習対象になる偏りを解消する。

## 2. 条件・検証

`graspnet.yaml` では、入力点数が 100000 点を超えても点列全域が GNG のボクセル化前処理対象になる。

- 追加 parameter: `input.sampling_mode` (`head` / `uniform`)
- topic と message field の変更はない。

- `docker exec gng_cpu_container_uraki ... colcon build --packages-select ais_gng --symlink-install`
- `graspnet.yaml` とダミー入力 topic を使った6秒間の初期化 smoke test

**制約**

- `uniform` は入力点列上の等間隔抽出であり、3D 空間上の等密度化ではない。
- GNG 内部のランダム学習選択は変更しない。
