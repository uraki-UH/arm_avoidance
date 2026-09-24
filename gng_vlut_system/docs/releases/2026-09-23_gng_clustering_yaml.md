# 2026-09-23 - センサー別YAMLからのPl・Curve切替

## 1. 要約

`ais_gng.launch.py`で選ぶセンサー別YAMLの`plane_clustering`・`curve_clustering`から、平面・曲面計算のON/OFFを指定可能。

- 共通設定よりセンサー別YAMLを優先。対応する明示launch引数の優先は維持。
- 利用者指定の短い切替名を内部パラメータへ変換。旧名との併記時は短い名前を優先。
- CPUの`plane_cluster.*`・`nonplane_component.*`を直結ノードへ転送。
- `surface_model.*`を曲面計算ノードへ転送。GNG側の時間通知購読先にもセンサー別設定を適用。

`at128.yaml`へ`plane_clustering: false`と`curve_clustering: false`を追加。既定OFFの動作を維持。

センサー別YAMLの指定が後段の共通設定に上書きされる問題と、同YAMLの曲面設定が計算ノードへ渡らない問題。

**削除**

at128.yamlの切替欄を短い名前へ変更。内部のROSパラメータ名と旧名の読込互換は維持。launch引数の削除なし。

## 2. 条件・検証

ログ文字列だけの表示切替ではなく、既存の計算ON/OFFへ接続。既にセンサー別YAMLへ同名設定がある環境では、今回からその値を優先。
未指定のセンサーは共通設定を継承。`at128.yaml`に値がある間は、共通設定だけの編集ではこの2項目は変化しない。
GNGコア内蔵の`Clusters`に対応するクラスタリングは対象外。

- `plane_clustering`: CPU直結の平面計算。内部は`plane_cluster.direct_enabled`。無効時は依存する非平面成分の計算も停止。
- `curve_clustering`: 別ノードの曲面計算。内部は`surface_model.enable`。通常CPU構成では平面計算も有効化が必要。
- 上記2項目はセンサー別YAMLの`ais_gng_node.ros__parameters`へ指定するlaunch用設定。ROSノード直接起動・実行中の`ros2 param set`用の別名ではない。値はYAMLの真偽値のみ。文字列`"false"`・数値・nullは起動前に拒否。
- `start_plane_cluster:=false`: 別ノードの起動抑止を引き続き優先。YAMLのCurve有効化だけでは解除しない。
- `surface_model.output_topic`: センサー別指定を計算側と時間通知購読側へ共通反映。
- トピック・メッセージ定義変更なし。設定変更後はlaunch再起動が必要。

launchのNodeアクションを生成し、最終パラメータを展開するテスト9件が成功。
共通設定の継承、平面・曲面ON/OFF全4組合せ、時間通知先の一致、明示引数優先、非平面設定、明示的な別ノード起動抑止を確認。
短名のON/OFF全4組合せ、旧名との併記時の優先、不正型の拒否も確認。
Nodeアクションの実行・実点群入力・計算結果の検証は未実施。C++変更・再ビルドなし。

上記は当初のPython辞書展開での確認。後続調査でROSのセレクター間の優先順位を再現していないことが判明。
実ROSによるテストへの置換と、入力トピック・明示引数・短名設定の優先順位の修正は[後続の検証記録](2026-09-23_gng_input_topic_override.md)を参照。

```bash
docker compose exec -T gng_cpu bash -lc 'source /ros2_ws/install/setup.bash && PYTHONDONTWRITEBYTECODE=1 timeout -s INT -k 5 30 python3 /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/test_clustering_yaml_launch.py -v'
```

上記プロセスは終了済み。ROSノード・デーモンの新規起動なし。既存GNG・rosbag・Viewerの停止・再起動なし。
インストール済みlaunchとat128.yamlの参照先が変更元ソースであることを確認。

**制約**

GPUではCPU直結の平面ON/OFF設定は無効。曲面設定の転送は両バックエンド共通だが、GPUの実行検証は対象外。
`Curve`は別ノードの時間通知状況を表示。設定trueでも通知前は`--`、別ノード未起動なら`off`となり、同一トピックの他publisherがあれば影響あり。
既存のセンサー設定値・他作業の差分は保持。
