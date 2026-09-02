# 2026-09-02 - 物体テンプレート照合対象のYAML選択

## 概要

物体テンプレート照合の読込対象を、単一の`dataset_file`固定から
`object_template_matching_sources.yaml`で変更できる方式へ拡張した。

## 設定

```yaml
template_sources:
  dataset_files:
    - basket_complete
    - /datasets/jug_complete_gng_template.json.gz
  dataset_dirs:
    - /datasets/kitchen
  exclude_dirs:
    - edge_repair_backup
  exclude_template_ids:
    - table
```

- `dataset_files`は個別のデータセットファイル、または`dataset_dir`配下の接頭名指定。
- `dataset_dirs`は`object_template`またはGNG同梱済み`object_surface_dataset`を再帰読込するフォルダ。
- `exclude_dirs`は`dataset_dirs`の再帰読込から除外するフォルダ。相対指定は`dataset_dir`基準。
- `exclude_template_ids`は個別指定・フォルダ指定の双方から同じ`template_id`を除外する。
- 初期YAMLはすべて空であり、フォルダ配下の全テンプレートを暗黙には読込まない。

## 挙動

- 選択テンプレート群は1つの`object_template_matcher_node`が読込む。
- matcherは起動時に、template GNG nodeの法線鉛直成分、`rho`、edge次数、隣接法線関係から逆引き索引を構築する。
- 入力時は最大128個の環境nodeと各key最大4templateだけを参照し、索引候補のうち1templateだけを全yaw探索で精密照合する。
- template数に比例する処理は起動時の読込・索引構築だけであり、入力frameごとの索引参照数と全yaw探索数は固定とする。
- validatorと静的GNG配信器はtemplateごとに起動する。
- 複数テンプレート時のnode名は`object_template_match_validator_node_<template_id>`、
  `object_template_map_publisher_node_<template_id>`となる。
- candidate、state、静的GNG topicは従来どおり`template_id`ごとに分離する。
- 同一`template_id`を異なるファイルから二重に選んだ場合は、起動前にエラーとする。
- `dataset_file`を指定した場合は、YAMLの選択を使わない単一テンプレート互換動作とする。

## 実行

`config/object_template_matching_sources.yaml`を編集してから、追加引数なしで起動する。

```bash
ros2 launch gng_vlut_system object_template_matching.launch.py
```

単体検証には従来どおり次を使用できる。

```bash
ros2 launch gng_vlut_system object_template_matching.launch.py \
  dataset_file:=basket_complete
```
