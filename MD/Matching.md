# 物体テンプレート照合

## 保存・静的配信

```bash
ros2 run ais_gng save_object_gng_dataset mug_complete --replace --with-points
ros2 launch gng_vlut_system object_template_map_publisher.launch.py dataset_file:=mug_complete
```

`--replace`は同名テンプレートと対応する過去の点群・画像を置換。保存先は`/datasets/mug_complete_gng_template.json.gz`。
履歴を残す場合は`--replace`を省略。

## 環境との照合・ファジー認識

```bash
ros2 launch gng_vlut_system object_template_matching.launch.py dataset_file:=mug_complete
```

設定は[object_template_matching.yaml](gng_vlut_system/config/object_template_matching.yaml)。
結果は`/object_recognition/classes`。追加ノードを未ビルドの場合は`gng_vlut_system`を再ビルド・source。

クラス階層・属性・代表テンプレートの対応は
[gng_vlut_system/config/object_class_recognition.yaml](gng_vlut_system/config/object_class_recognition.yaml)で管理。
`templates`のキーは保存済みJSONの`template_id`。車両テンプレートは事前生成・保存と
[照合対象の選択](gng_vlut_system/config/object_template_matching_sources.yaml)が必要。
設定にクラス名を追加しただけでは未登録のモデルを認識できない。

- `hypotheses`: 個別テンプレート候補ごとの結果。
- `classes` / `attributes`: シーン内の存在根拠。大型・箱型などの属性も同時出力。
- 観測不足や更新停止: `membership: null`。未検出を適合度0で代用しない。
- 属性: 代表テンプレートの注釈由来。実寸法の測定ではない。
- 適合度: 未校正のファジー評価。確率や認識精度ではない。
- 初期検証: 箱の車判定、車種間の重複支持、取っ手のない円筒のマグカップ判定を確認。現状の適合度だけで細分類を確定しない。

追加ノードの無効化は`enable_class_recognition:=false`。
[評価仕様・状態・制限](gng_vlut_system/docs/TECHNICAL_SPEC.md#ファジークラス属性認識)。
