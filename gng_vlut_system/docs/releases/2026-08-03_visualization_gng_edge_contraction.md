# 2026-08-03 - visualization GNG edge contraction

## Summary

可視化GNGのedgeを、3次元GNGが再学習したedgeから元姿勢GNGのcoord-space edgeを
縮約したedgeへ変更した。可視化ノードの位置と元ノード所属の学習方法は変更しない。

## Changed

- activeな元ノードごとに、対象coord layerの隣接元ノードIDを収集する。
- 元edge両端の所属可視化ノードを接続し、自己loopと重複edgeを除去する。
- `source_signature` schema 2は元ノードID、3次元座標、coord-space edgeを照合する。
- trainerの保存後検証へ縮約edge一致、連結成分数、孤立ノード数を追加した。

## Fixed

可視化ノード配置だけからedgeを再学習していたため、元GNGの接続関係が表示から欠落し、
edgeが途中で切れて見える問題を修正した。

## Behavior Impact

bin形式はversion 1のままだが、signatureの計算対象へcoord-space edgeを追加した。
旧signatureの可視化binはbridgeで不一致として配信されないため、trainerで再生成が必要になる。
距離によるedge除去やk近傍edgeの追加は行わず、元GNGの位相をそのまま縮約する。

## Topics / Params / Messages

topic、ROS parameter、message schemaの変更はない。

## Measured Result

- ToPoDualArm3: 941元ノード、500可視化ノード、2,295edge、1連結成分、孤立0
- ToPoDualArm3 bin: 30,156 bytes
- ToPoDualArm10000: 10,801元ノード、500可視化ノード、2,504edge、1連結成分、孤立0
- ToPoDualArm10000 bin: 71,268 bytes
- edge長最大値: ToPoDualArm3 0.542693m、ToPoDualArm10000 0.510962m

## Verification

- Humbleコンテナ内で`gng_vlut_system`全体build成功。
- 両モデルをtrainerで再生成し、元ノード所属、縮約edge、連結性を保存後に再読込検証。
- ToPoDualArm3を同一seedで一時出力へ再生成し、`cmp`で完全一致。
- 隔離ROS domainのbridgeで500ノード、2,295縮約edgeの読込とtopic初期化を確認。
- テスト用node、trainer、build processが残っていないことを確認。
- `git diff --check`。

## Risk / Notes

元GNGに長いcoord-space edgeがあれば可視化GNGにも残る。今回の変更は位相欠落の修正であり、
長さによる除去は元位相を変えるため追加していない。
