# 2026-08-12 - HTML gripper volume graph input

## Summary

`ToPo-FUZZY_Manipulation_v1.html`がグリッパー体積graphをライブROSとrosbag bundle JSONから
受け取り、今後の把持候補評価計算から参照できるようにした。

## Changed

- rosbridgeを単体websocket nodeではなく、`rosapi`も含む公式launchで起動する。
- ToPoDualArm用のgrasp bundle設定へ左右の体積graph topicを追加する。

## Added

- 命名規則とmessage型による任意数のグリッパー体積graph topic自動購読。
- topicごとに正規化した`nodes`、`edges`、`clusters`、`frameId`を共通の評価入力ストアへ保持。
- `window.__topoEvaluationMetricApi`によるstructured inputの登録・検索・bundle復元API。
- HTML上の評価入力ステータスへschema、sample、structured inputの件数をまとめて表示。

## Behavior Impact

- `/evaluation_metrics`は評価値専用のままで、graph構造を格納しない。
- 体積graphはメインGNG graphを上書きせず、`evaluationMetrics.structuredInputs`として保持する。
- ライブROSではtopic追加後、最大5秒以内に自動発見して購読する。

## Topics / Params / Messages

- Message: `ais_gng_msgs/msg/TopologicalMap`
- ToPoDualArm既定topic: `/ToPoDualArm/L_grip_V_topological_map`、`/ToPoDualArm/R_grip_V_topological_map`
- rosbag bundle role: `gripper_volume_graph`
- 評価値topic: `/evaluation_metrics`（変更なし）

## Verification

- HTMLの全inline scriptをNode.js `vm.Script`で構文解析: 4/4成功。
- headless Chromeから単体HTMLを開き、rosbridge接続成功。
- rosapi自動発見で左右2 topic、1008 nodes、2642 edges、2 clustersを受信。
- 左右frameが`ToPoDualArm/L_tcp`、`ToPoDualArm/R_tcp`、各中心Zが0.04415 mであることを確認。
- 合成bundle JSONから任意topic/frameのgraph、node、cluster、中心、scaleを復元。

## Risk / Notes

- ライブ自動発見には`rosapi`が必要。`rosbridge_websocket`単体起動では自動発見できない。
- rosbagには記録対象topicを明示する必要がある。別ロボットではexporter設定へ同じroleで追加する。
- TFによるworld座標変換や体積graphを使う具体的な評価式は、この入力ストアとは分離して追加する。
