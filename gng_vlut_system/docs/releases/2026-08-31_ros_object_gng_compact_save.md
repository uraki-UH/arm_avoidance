# 2026-08-31 - ROS物体GNGの圧縮保存と元点群分離

## 概要

ROSで学習した物体GNGをcompact JSONのgzipとして保存し、必要な場合だけ元`PointCloud2`を
binary-compressed PCDへ分離保存する方式へ変更。

## 変更

- 保存サービス名を`/save_gng_data`へ変更
- 保存時の指定を物体名だけとし、UTC日時と物体別通し番号を自動付与
- `object_surface_dataset`の空ラッパーを廃止し、`object_template`を直接保存
- 空配列、既定値、空の`inpcl_ids`を省略
- edgeをnode添字の2要素配列として保存
- テンプレートを`*_gng_template_v1.json.gz`へgzip保存
- 任意の元点群を`*_source.pcd`へbinary-compressed PCDとして分離保存
- 点群の相対ファイル名、topic、frame、stamp、点数、field定義、色情報の有無をテンプレートへ記録
- 静的マップ配信ノードと単体HTMLへgzip読込対応を追加

## 操作

GNGだけを保存:

```bash
ros2 run ais_gng save_object_gng_dataset concrete
```

同じstampの元点群も保存:

```bash
ros2 run ais_gng save_object_gng_dataset concrete --with-points
```

元点群保存には`ais_gng.launch.py`の`source_point_cloud_topic`指定が必要。`auto`は明示された
`input_topic`を使用し、空文字は点群保存を無効化。
