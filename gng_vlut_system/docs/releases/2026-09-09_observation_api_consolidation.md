# 2026-09-09 - 観測APIの3関数への整理

## Summary

観測入力の一括設定、フレーム情報取得、ノード角度範囲取得の3関数へ整理。
原点と画素参照の個別設定順序への依存を解消。観測支持や境界判定の方式変更なし。

## Changed

- `gng_set_observation_input(const gng_observation_input *input)`：原点と任意の画素ビュー・角度表の一括設定。
- `gng_get_observation_frame()`：原点・フレーム番号に加え、`pixel_hit_num`・`ray_num` の取得。
- `gng_get_observation_angle_range(node_id)`：従来どおりのノード単位の取得。
- ROS側は原点と画素参照を準備後、公開APIを一度だけ呼ぶ方式。
- 旧画素配列専用の内部ポインタ・個数・学習時分岐を削除。連続配列もpixel形式のビューで入力可能。

## Added

- 一括入力構造体 `gng_observation_input`。
- 旧設定の残存、原点のみへの置換、無効原点、null入力、無効な表・ビューの回帰検査。
- 次入力の設定後も直近出力の原点・フレーム番号・参照件数を保持する検査。

## Fixed

原点だけの設定呼び出しによって、先に設定した画素参照が意図せず消える呼び出し順依存。

## Removed

- `gng_set_observation_origin`
- `gng_set_observation_pixels`
- `gng_set_observation_pixel_view`
- `gng_get_observation_lookup_statistics` と専用返却構造体

上記の互換ラッパー・公開シンボルは維持しない方針。利用側は新APIへの移行が必要。

## Behavior Impact

- `gng_setPointCloud` → `gng_set_observation_input` → `gng_exec` の順序。
- 設定ごとの一括置換。原点のみの設定では以前の画素参照を継承しない挙動。
- `nullptr`、`has_origin=0`、非有限原点、機能OFFは受付結果0かつ観測入力の解除。
- 有効原点で画素ビュー未指定の場合は受付結果1、既存レイ計算の利用。
- 無効ビュー・入力点数不一致・無効表は受付結果0、旧画素参照の破棄と今回の有効原点によるレイ計算への復帰。
- 構造体とビューは値コピー。元点群・選択番号・角度表は借用、`gng_exec` 完了まで保持が必要。
- 次入力・学習完了・支持設定変更で借用は失効。次入力設定による直近出力の変更なし。
- 追加の全点コピー・全点探索なし。次数による境界候補判定への変更なし。

## Topics / Params / Messages

ROSトピック・パラメータ・メッセージ定義の追加変更なし。
`node_observation_support` のversion=5、統計トピックの6要素も従来どおり。

## Verification

既存コンテナ `gng_cpu_container` の隔離先 `/tmp/gng-observation-api.GnXKN5` で検証。
通常の `/ros2_ws/install` への上書きなし。

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
cd /ros2_ws
timeout --signal=INT --kill-after=15s 600s colcon \
  --log-base /tmp/gng-observation-api.GnXKN5/log build \
  --build-base /tmp/gng-observation-api.GnXKN5/build \
  --install-base /tmp/gng-observation-api.GnXKN5/install \
  --packages-select gng_cpu ais_gng --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DGNG_BUILD_BENCHMARKS=ON -DBUILD_TESTING=ON
timeout --signal=INT --kill-after=5s 90s ctest \
  --test-dir /tmp/gng-observation-api.GnXKN5/build/gng_cpu --output-on-failure
nm -D --defined-only /tmp/gng-observation-api.GnXKN5/install/gng_cpu/lib/libgng_cpu.so | grep observation
timeout --signal=INT --kill-after=5s 60s ctest \
  --test-dir /tmp/gng-observation-api.GnXKN5/build/ais_gng -R '^test_observation_pixels$' --output-on-failure
```

- gng_cpuの次数・整数角度範囲・観測APIの3テスト成功。
- 共有ライブラリの観測公開シンボルが3個だけであることを確認。
- gng_cpu・ais_gngの2パッケージのビルド成功。既存コード・依存ライブラリ由来の警告あり。
- ROS型の画素ビュー単体テスト成功。初回はROS環境未読込によるテストラッパー失敗、読込後に成功。

ROS実行検証の起動コマンド（コンテナ内）：

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
source /tmp/gng-observation-api.GnXKN5/install/local_setup.bash
export LD_LIBRARY_PATH=/usr/local/lib/python3.10/dist-packages/torch/lib:$LD_LIBRARY_PATH
timeout --signal=INT --kill-after=10s 65s python3 \
  /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/observation_support_ros_test.py \
  --executable /tmp/gng-observation-api.GnXKN5/install/ais_gng/lib/ais_gng/ais_gng_cpu
timeout --signal=INT --kill-after=10s 65s python3 \
  /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/observation_support_ros_test.py \
  --executable /tmp/gng-observation-api.GnXKN5/install/ais_gng/lib/ais_gng/ais_gng_cpu \
  --transform-cloud --fixed-origin
```

- ROSドメイン187、専用入力トピック `/observation_test_points` で2条件とも成功。
- TF由来原点、固定原点と回転・並進、TF欠落・時刻不一致、実測レイに対応する範囲・配信形式を確認。
- 画素表経路はAPI・画素ビュー単体テストで検証。実カメラ入力による画素表配信検証は今回未実施。
- ビルド・テストはすべて終了済み。テスト用GNGの停止とプロセス残存なしを確認。
- 本作業からの既存GNG・Viewer・HTTPサーバー・ROSデーモンの停止や再起動なし。既存4コンテナの稼働維持。
- 最終確認時に本作業外でのGNG起動条件変更とbag再生追加を観測。その状態は変更せず、検証用プロセスの不在のみ確認。

## Risk / Notes

旧関数削除と `gng_observation_frame` の構造変更を伴うABI非互換。
ライブラリと利用側の双方の再ビルドが必要。旧バイナリと新ライブラリの混在不可。
既存の未コミット変更を保持し、今回の対象範囲のみ変更。
