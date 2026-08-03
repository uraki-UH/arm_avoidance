# 2026-08-03 - Include terminal branch links in VLUT

## Summary

TCPの兄弟リンクとして接続されたグリッパ指が姿勢VLUTから欠落していた問題を修正した。

## Changed

- profileのEEFから親リンクをたどり、同じ親から分岐する終端リンクとその子孫を
  自動的にVLUT対象へ含める。
- 直列KinematicChainにない分岐リンクのtransformをURDF joint treeから補完する。
- 分岐関節の既定値は0とし、mimic jointの倍率とoffsetを適用する。
- ボクセル衝突判定とVLUT構築で同じ分岐transform補完処理を使う。

## Fixed

`ToPoDualArm10000`の`L_finger_left`と`L_finger_right`が`vlut.bin`へ収録されるようにした。

## Behavior Impact

既存の7自由度GNG、手先座標、angle/coord edge、`L_link1`の明示除外は変更しない。
VLUTの関係レコード数とファイルサイズは、指の占有ボクセル分だけ増加する。

## Verification

- 隔離build/installで`gng_vlut_system`のビルドに成功。
- `vlut_only:=true`で既存10,801ノードの`gng.bin`からVLUTを再生成。
- `L_finger_left`: 308,782関係。
- `L_finger_right`: 308,649関係。
- 全関係数: 2,875,809。全10,801ノードを収録。
- 1ノードあたりの関係数: 最小243、中央値266、最大293。
- 再生成した`vlut.bin`: 57,516,224 bytes。

## Risk / Notes

GNG対象外のグリッパ開閉関節は既定値0で固定される。開度ごとの占有形状が必要な場合は、
グリッパ関節をGNG姿勢へ含めるか、複数開度を保守的に統合する別仕様が必要になる。
