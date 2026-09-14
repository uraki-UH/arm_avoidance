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