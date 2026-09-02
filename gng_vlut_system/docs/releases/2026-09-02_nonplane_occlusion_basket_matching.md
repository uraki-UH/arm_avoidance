# 2026-09-02 - 非平面成分を使う遮蔽basket照合

## 概要

登録済みbasketテンプレートと遮蔽された環境GNGの照合で、平面クラスタに接続する非平面成分を肯定証拠として利用する。
持ち手のように平面から伸びる小さな成分が一部だけ観測された場合でも、欠落を反証にせず候補を維持する。

## 変更

- テンプレートGNGから、平面クラスタ非所属nodeの連結成分と接続先平面クラスタを導出
- 環境`TopologicalMap`の`nonplane_component_id`から同じ構造を導出
- 平面対応が一致する非平面成分について、法線・`rho`・内部edge密度を使う証拠scoreを算出
- 候補JSONへ非平面成分数、対応、証拠score、観測有無を追加
- validatorで、観測済み非平面成分の証拠が5フレーム連続で得られた場合だけ確定条件へ利用
- `enable_roll_pitch_search: false`のとき、roll/pitchを確実に0度固定
- 遮蔽入力用profileで、非平面証拠の重みと飽和スケールを調整
- 遮蔽時の候補可視率下限を`0.15`へ調整

## static mapの挙動

`/<template_id>/topological_map_static`はvalidatorの状態が`confirmed`になるまで配信しない。
Viewerではソース一覧から対象topicを有効化して初めて購読・描画する。

このtopicは登録テンプレートの座標系`object_template`で配信する静的参照グラフである。
照合結果の位置・姿勢を環境座標系へ登録した物体仮説グラフではないため、環境点群の`x=-0.4 m`やyawへ重ねて描画する処理は別途必要になる。

## 検証

- `gng_vlut_system`のReleaseビルド成功
- `/topological_map`と`/plane_clusters`を入力に、basket候補で非平面成分対応を受信
- 既定の5フレーム連続条件で`confirmed: true`を確認
- 確定状態に連動して隔離topicのstatic mapから`279 nodes`を受信
- 検証用matcher、validator、static map publisherはすべて停止済み
