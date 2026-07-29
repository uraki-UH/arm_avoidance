# 2026-07-29 - link manipulability frame alignment

## Summary

ToPoFuzzy Viewer のリンク別可操作性楕円を、選択した URDF リンクの位置と姿勢へ一致させた。

## Changed

- マルチアーム内部チェーンの共有関節を、同名関節を持つすべての腕へ反映
- リンク別可操作性楕円の姿勢を world/base 相対から対象リンク相対へ変更
- Viewer ではリンク別楕円をロボットルート直下ではなく対象 URDF リンク直下へ描画
- 回転可操作性に専用の `rotationalManipCenter` を使用

## Added

- 共有関節名から複数の内部 DOF 添字への対応
- `RobotData`、`RobotPoseInstance`、`RobotLinkManipulability` の `rotationalManipCenter`

## Fixed

- `waist_joint` などの共有関節が片側の内部チェーンにしか反映されない問題
- `L_tcp` を選択しても楕円中心がリンク位置と一致しない問題
- 手動関節表示でリンクだけが動き、楕円中心がライブ姿勢に残る問題
- 回転楕円が並進楕円の中心フィールドを常に参照していた問題
- 楕円クリック後にリンク選択を変更すると、選択表示と実際の描画対象が食い違う問題
- 指定リンクの評価データがない場合に、別対象のトップレベル可操作性を描画する問題

## Removed

- リンク別楕円をロボットルート座標へ配置するための中心座標変換

## Behavior Impact

- リンク別楕円の中心は常に選択リンクの原点になる
- 手動表示では楕円の位置と姿勢が選択リンクへ追従する
- 手動表示時の楕円スケールは ROS から最後に受信した可操作性評価値であり、ブラウザ側では Jacobian を再計算しない
- リンク別 `manipOrientation` と `rotationalManipOrientation` は対象リンク相対として解釈する

## Topics / Params / Messages

- Topic 名、parameter、ROS message の変更なし
- Viewer 内部 JSON のリンク別可操作性姿勢の座標系を対象リンク相対へ変更

## Verification

- `colcon --log-base /tmp/codex_gng_log3 build --packages-up-to gng_vlut_system --build-base /tmp/codex_gng_build3 --install-base /tmp/codex_gng_install3 --cmake-args -DCMAKE_BUILD_TYPE=Release`
- `npx tsc -b --pretty false`
- `npx vite build --configLoader runner --outDir /tmp/topofuzzy-manip-build --emptyOutDir`
- `git diff --check`
- 稼働中の旧 bridge では右腕と腰が固定でも、左腕の更新ごとに `R_tcp` の中心と姿勢が変化することを WebSocket 上で確認
- `/ros2_ws` を再ビルドして `gng_viewer_bridge.launch.py` を再起動後、`R_tcp` 中心が `[0, 0, 0]`、リンク相対姿勢が 454 frame 連続で不変であることを確認

## Risk / Notes

- ROS ノードと Viewer の両方を同時に更新する必要がある
- 通常の frontend `dist/` は既存ファイルの所有権により更新できなかったため、検証ビルドは `/tmp` へ出力した
