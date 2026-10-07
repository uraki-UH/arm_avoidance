## HTML全点群からCPU GNGテンプレートを保存
点群も保存
source /ros2_ws/install/setup.bash
ros2 run ais_gng save_object_gng_dataset mug_complete  --replace --with-points

--replaceをつけると同名で保存していたやつ削除

保存先は`/datasets/設定名_<UTC日時>_<連番>_gng_template.json.gz`。

同名テンプレートを置換し、過去の同名保存と対応する点群・深度・色情報を削除する場合。


置換保存先は`/datasets/mug_complete_gng_template.json.gz`。

保存済みテンプレートは、保存名の接頭名だけで静的トピックへ配信。

source /ros2_ws/install/setup.bash
ros2 launch gng_vlut_system object_template_map_publisher.launch.py \
  dataset_file:=mug_complete

## 環境GNGとの照合後に物体テンプレートを配信
ros2 launch gng_vlut_system object_template_matching.launch.py \
  dataset_file:=mug_complete

姿勢許容、特徴量のファジー評価、確定条件は
`/ros2_ws/src/gng_vlut_system/config/object_template_matching.yaml`で設定する。

## クラス・属性のファジー認識

`object_template_matching.launch.py`で同時起動。初回は追加ノードのinstall反映が必要。

```bash
cd /ros2_ws
colcon build --packages-select gng_vlut_system --symlink-install
source /ros2_ws/install/setup.bash
ros2 launch gng_vlut_system object_template_matching.launch.py dataset_file:=mug_complete
```

結果の確認:

```bash
ros2 topic echo /object_recognition/classes
```

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
