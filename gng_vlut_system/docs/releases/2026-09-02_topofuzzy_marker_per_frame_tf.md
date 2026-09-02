# 2026-09-02 - TopoFuzzy MarkerArray個別TF

## 概要

TopoFuzzy Viewerの`visualization_msgs/msg/MarkerArray`表示を、配列単位ではなく各`Marker.header.frame_id`単位でTF変換する方式へ変更した。

## 変更内容

- WebSocket受信時にMarkerごとの`frameId`を保持し、個別の親groupへ対応するTFを適用する。
- 配列内に複数フレームが混在する場合も、各Markerを正しいフレームで描画する。
- TF未受信の非`world`フレームは原点への誤表示を避けるため、受信まで描画を保留する。
- 静的TFは子フレームID単位で統合して再接続時に再送する。複数の`static_transform_publisher`が存在しても最後の1件だけで上書きしない。
- MarkerArrayの最新payloadをbackendで保持し、WebSocket再接続時にも再送する。
- レイヤー表示に単一フレーム、混在フレーム、TF未受信状態を反映する。
- `viewer_edit_node`は子フレームごとに時刻が後退した動的TFをBuffer投入前に破棄する。
- `map`と`world`を固定ルートとして扱い、親TFがない`map`座標のMarkerArrayも表示する。

## 挙動影響

- LiDAR座標系で発行される平面クラスタのMarkerArrayは、`world`への静的TF受信後にGNGノード・エッジと同じ姿勢で描画される。
- 複数フレームを含むMarkerArrayは、配列の先頭Markerのフレームへ誤ってまとめて変換されない。
- 古い`/tf`再送は編集用TF Bufferへ入らず、`TF_OLD_DATA`警告を出さない。
- `frame_id: map`の`semantic_points`由来平面クラスタは、`world → map` TFがなくても表示する。
- topic、ROS parameter、message定義の変更なし。

## 検証

- `./node_modules/.bin/vite build --configLoader runner --outDir /tmp/topofuzzy-viewer-build`: TypeScript型検査とproduction build成功。
- `npm run lint`: 今回の変更によるerrorなし。既存の`ObjectTemplateMatchTuner.tsx`にReact Hook dependency warning 1件。
- `git diff --check`: 成功。
- Docker backend buildは依存PCL全体の再コンパイルへ進んだため、既存実行環境への負荷を避けて中止。今回起動したbuildプロセスは停止済み。

## 注意

- 動的TFの時刻補間は既存どおりViewerが受信した最新TFに基づく。`Marker.header.stamp`時刻での厳密な再生はrosbag時刻同期の別機能として扱う。
