# 2026-08-31 - HTML物体テンプレートのファイル指定配信

## 概要

`ToPo-FUZZY_Manipulation_v1.html`で保存した完全表面点群テンプレートを、
`object_template_map_publisher.launch.py`から直接配信できるようにする変更。

## 変更

- HTMLの`template_id`をROS topic名に使用可能な英字開始・英数字・`_`形式へ自動採番
- `dataset_file`引数によるJSON、gzip JSON、接頭名の指定
- 接頭名が一意に一致するHTML保存データセットの自動選択
- 既存の`dataset_id`引数の互換維持

## 操作

```bash
ros2 launch gng_vlut_system object_template_map_publisher.launch.py \
  dataset_file:=mug_complete_v1
```

`/datasets`内に`mug_complete_v1`で始まるテンプレートファイルが1件だけある場合、そのファイルを配信対象とする。
