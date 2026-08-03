# 2026-08-03 - precomputed visualization trajectory interpolation

## Summary

元GNGのangle-space edgeを関節角補間し、URDF FKで得た手先軌道を可視化GNGノード列へ
事前変換してbinへ保存するようにした。bridge実行時のFKと最近傍探索は行わない。

## Changed

- 可視化GNGの静的edgeをcoord-space edge縮約から、angle-space edgeのFK補間遷移へ変更。
- 可視化binを`VIZGNG2`へ更新し、補間経路overrideを追加。
- source signature schemaを3へ更新し、関節角とangle-space edgeも照合対象に追加。
- bridgeの元軌道変換を、所属先の直接接続から保存済みedge pathの表引き展開へ変更。

## Added

- `--interpolation-joint-step`: 補間1区間の最大関節差。既定値0.05 rad。
- `--max-interpolation-samples`: 元edge 1本あたりのサンプル上限。既定値256。
- 元edgeから補間列への起動時hash index。

## Behavior Impact

元軌道の1 edgeが複数の可視化edgeへ展開されるため、可視化軌道のノード数は元軌道より
多くなり得る。bridgeの変換量は入力edge数`L`と出力ノード数`K`に対して`O(L + K)`。
直接接続だけで表せるedgeはbinへ経路を保存せず、所属対応から実行時に復元する。

version 1 binは読み込まない。元`gng.bin`とROS params YAMLから再生成が必要である。

## Topics / Params / Messages

既存のplanned/candidate入出力topicと`ais_gng_msgs/msg/TopologicalMap`は変更しない。
生成時は`--ros-args --params-file <yaml>`を指定し、URDF、profile、選択関節を取得する。

## Verification

- Humbleコンテナ内で`gng_vlut_system`全体build成功。
- ToPoDualArm3: 941元ノード、500可視化ノード、3,108 edge、26,161 override、
  1連結成分、孤立0、650,498 bytes。
- ToPoDualArm10000: 10,801元ノード、500可視化ノード、3,187 edge、
  219,432 override、1連結成分、孤立0、4,135,260 bytes。
- 保存直後の再読込で全所属、全edge、全補間列の一致を検証。
- 分離ROS domainで元edge `0 -> 1`を入力し、保存済みの中間可視化ノードを含む
  6ユニークノード、6 edgeへ展開されることを実受信。
- テストbridgeを停止し、テストが追加したdomain 226のROS daemonを削除。
- コンテナの起動状態と既存domain 25 daemonが開始前と一致することを確認。

## Risk / Notes

binはx86_64 little-endianのnative binaryを前提とする。可視化ノードIDと補間中間IDは
`uint16`で保存する。`TopologicalMap`は再訪ノードを統合するため、edge集合は描画できるが
厳密な再生順序が必要な場合は順序付き経路messageを別途追加する必要がある。
