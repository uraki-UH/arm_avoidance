ais_gngについてすでにこの中にボクセル化処理があるので、world_indexbucketの処理とかでもボクセルに点群登録している処理が重複している気がしていてどうにかできる？

できます。ただし ais_gng の内部VoxelGridをそのまま world_index_bucket に流用するのは危険です。
現時点の実行中Dockerには world_index_to_voxel_node がいないため、今この起動構成では重複は発生していません。
役割が異なります。
- ais_gng のVoxelGrid: GNG学習用のフレーム内・固定格子で、各セルを平均化して入力点を減らす処理。[voxel_grid.cpp (line 21)](/home/fuzzrobo/uraki_ws/ais_gng_cpu/src/gng_cpu/src/cpu/voxel_grid.cpp#L21)
- world_index_bucket: world座標へ変換した元点群を粗いbucketへ登録し、複数ロボットROIごとに必要な点だけ取り出してから衝突用voxel IDへ変換する処理。[world_index_to_voxel_node.cpp (line 619)](/home/fuzzrobo/uraki_ws/gng_vlut_system/src/nodes/bridge/world_index_to_voxel_node.cpp#L619)
GNG用の平均点だけをworld indexに渡すと、ROI・衝突判定で必要な細部が失われます。またGNG内部の格子はprivateなフレーム内バッファで、world indexはworld座標・複数ROI・別解像度を前提にします。
推奨は、内部Gridを共有するのでなく、入力直後に共有の階層点群インデックスを一回だけ作る方式です。
1. PointCloudをworld座標へ一回だけ変換
2. 同じ走査で粗いworld bucketと、GNG用の細かいvoxelセルを同時構築
3. world indexは各bucketに元点への参照を保持
4. GNGは細かいセルの重心と、元点ID範囲を受け取る
5. ROI側は粗いbucketから元点を取り出し、必要なROIだけ最終voxel化
bucket_size=0.2 とGNGの input.voxel_grid_unit=0.02 のように整数倍で揃えると、粗いbucketを親、GNG voxelを子にした階層にできます。
短期的には ais_gng の /scan/transformed をworld indexの入力にしてTF変換の重複だけ減らせます。ただしvoxel化自体は二重のままです。
完全統合では、GNGへ「事前voxel化済み入力」を渡す狭いAPIを追加して、内部VoxelGridをbypassするのが保守性も良いです。

ガウシアンミクスチャーモデルみたいなの使える？なんかガウシアンスプラっティングみたいだけどね







9:32 PM






使えます。むしろ現在のGNG nodeは、GMMの成分にかなり自然に対応します。
\[
p(x)=\sum_i \pi_i \mathcal{N}(x;\mu_i,\Sigma_i)
\]- 平均 μ_i: GNG node位置
- 共分散 Σ_i: 勝者点群共分散を基礎にする
- 重み π_i: 勝者点数、または局所表面面積に基づく重み
ただし、勝者点群共分散だけをそのまま使うのは不安定です。GNG密度や学習状態で小さくなり過ぎるので、照合用には
\[
\Sigma_i^{match} =
\Sigma_i^{winner}
+ \Sigma_i^{kernel}
+ \Sigma_i^{sensor}
\]のように、物理スケールで決めた最小カーネルを足すのがよいです。
- 接平面方向: 少し広めの許容幅
- 法線方向: 深度ノイズ・表面厚み用の小さい許容幅
- 固有値に下限を設ける
- テンプレート全体で一つのスケール倍率範囲を持たせる
欠損に強い照合
環境側の成分 j をテンプレートGMMに片方向で評価します。
\[
S =
\sum_j w_j
\log \left(
\sum_i \pi_i
\mathcal{N}(
\mu_j;\,
\mu_i,\,
\Sigma_i+\Sigma_j
)
\right)
\]これなら、環境で観測できた面がテンプレートのどこかに説明されれば高得点になります。見えていないテンプレート側の面は罰しないので、遮蔽・欠損に対応しやすいです。
yawだけ見るなら、各yaw候補でテンプレートの平均・共分散を R_z で回します。
\[
\mu_i' = R_z\mu_i,\quad
\Sigma_i' = R_z\Sigma_iR_z^\top
\]平行移動を推定しない方針なら、各クラスタを自分の重心や基準node中心へ正規化してから評価します。これは位置合わせの探索ではなく、座標原点を消す前処理です。
Gaussian Splattingとの違い
- 3D Gaussian Splatting: 画像レンダリング向け。色・不透明度・視点依存表現を持つ
- 今回のGMM: 3D空間上の確率密度・形状照合向け。色や画像レンダリングは不要
Viewerでは同じ楕円体を表示できますが、照合本体は画像化せずガウス尤度を直接計算する方が軽くて保守しやすいです。
ただしGMMだけではedge接続や部位構造が弱いので、最終スコアは「GMM尤度 + 法線/edge不変特徴 + 階層ボール整合性」にするのがよいです。

## 物体GNGテンプレート照合と非平面特徴抽出の作業方針

### 目的

事前登録した物体GNGテンプレートと、深度または環境側点群から学習したGNGを照合する。
環境側は遮蔽、視野外、GNG node密度差、法線符号反転を含むため、テンプレート全体の完全一致を
要求しない。観測された特徴からテンプレート仮説を起動し、観測済みの局所構造だけで検証する。

当面はyawを主探索自由度とし、平行移動は照合の探索対象に含めない。
roll・pitchはテンプレートごとの許容姿勢としてファジー評価する。位置中心合わせは、テンプレートと
環境データが同一形状を表すか確認する診断専用処理とする。

### 姿勢許容のファジー評価

yaw候補ごとに、対応した平面法線と非平面特徴の局所座標系からテンプレートに対する相対姿勢を推定する。
その相対姿勢を`R = R_z(yaw) R_y(pitch) R_x(roll)`として分解し、`abs(roll)`と`abs(pitch)`を
独立に評価する。符号反転した法線を許容するため、roll・pitchの推定は単一法線では確定せず、
複数平面または非平面特徴との整合から求める。

船など水平設置を想定するテンプレートの初期設定は、次のようにする。

```yaml
pose_feature:
  yaw:
    enable_search: true
    min_deg: -180.0
    max_deg: 180.0
    step_deg: 5.0
  roll:
    max_full_match_deg: 5.0
    max_allow_deg: 20.0
  pitch:
    max_full_match_deg: 5.0
    max_allow_deg: 20.0
```

- `max_full_match_deg`以内: 姿勢制約として満点
- `max_full_match_deg`から`max_allow_deg`: 角度超過量に応じた連続的な減点
- `max_allow_deg`超過: 姿勢制約による候補の反証

roll・pitchの上限は物体種別ごとに変える。設置状態が不定な物体は大きな上限または評価無効を選び、
船のように重力方向が意味を持つ物体は小さく設定する。姿勢推定に必要な特徴が欠損した場合は、
未観測として姿勢制約の減点・反証を行わない。

### 現在の実測

対象は遮蔽なし`basket`点群へZY並進とyaw回転を与えて学習した環境GNGと、
`basket_gng_template.json.gz`とする。

| 項目 | テンプレート | 環境 |
|---|---:|---:|
| 入力点数 | - | 4,200 |
| GNG node数 | 279 | 392 |
| GNG edge数 | 665 | 917 |
| 平面クラスタ数 | 5 | 5 |

診断専用の中心合わせyaw走査では、GNG位置から46度、平面クラスタから47度、
符号不変法線だけの10度刻み照合から50度が得られた。三者は同じyaw近傍を示す。

| 指標 | 実測値 |
|---|---:|
| 対称Chamfer距離 | 6.476 mm |
| 最近傍距離中央値 | 約6 mm |
| 空間近傍で対応したtemplate edge | 342 / 665 = 51.4% |
| 対応edge方向差中央値 | 17.8度 |
| 対応edge長比中央値 | 0.912 |
| 法線符号反転率 | 40.1% |
| 符号不変法線角度中央値 | 10.8度 |

環境側には余剰nodeと外れedgeがある。最大edge長はテンプレート約29 mmに対して
環境約178 mmである。node IDやedge接続の完全一致を照合条件にしてはいけない。

### 現在の保存データ

テンプレートJSONには次を保存する。

- `gng.nodes`: node位置、法線、`rho`、勝者残差サンプル数、勝者残差共分散
- `gng.edges`: node添字対
- `gng.plane_clusters`: 平面重心、法線、extent、平面性、残差率、spacing、位置共分散

`winner_point_covariance`は所属点群の位置分散ではない。学習時の
`input_point - winner_node_position`の残差共分散、すなわち勝者nodeの誤差楕円体である。
現状は残差平均を保存していないため、系統誤差の中心は復元できない。

### 安定指標の優先順位

1. 平面クラスタ集合の符号不変法線とyaw
2. 平面間の相対構造
3. 局所edge長列、edge長比、edgeのZ差分
4. 隣接node法線の符号不変内積
5. 勝者残差共分散の固有値比と局所edge長で正規化した固有値
6. `rho`とnode次数の補助ファジー評価

法線は表裏を区別しない方向特徴とする。

```text
normal_angle = acos(abs(dot(normal_template, normal_environment)))
```

絶対node位置、符号付き法線、`winner_point_count`、絶対共分散値、完全なedge接続は
主特徴として使わない。`winner_point_count`は入力密度と学習時間へ依存するため、
観測信頼度または反証の支持量としてだけ使う。

### 平面クラスタによる上位構造

`basket`ではテンプレート5平面と環境5平面がすべて対応した。符号不変法線角度は
0.12度から2.42度、extent比は0.645から1.206だった。

平面クラスタで使う特徴は以下とする。

- 符号不変法線角度
- extentのソート済み比、縦横比、面積相当値
- `planarity`、`residual_ratio`、`local_spacing`による信頼度
- 平面対ごとの絶対法線内積、重心間距離、重心差のZ成分
- 平面間のGNG接続関係

部分観測では平面extentが縮む。extentと面積は厳密一致にせず、片側欠損を許容する
ファジー評価とする。環境にある無関係な平面を反証に使わないよう、対象GNG成分と接続する
平面だけを仮説検証の対象にする。

### 平面サイズの非対称ファジー評価

環境平面のソート済みextentと面積相当値をテンプレート平面と比較する。比率は以下とする。

```text
extent_ratio_i = environment_sorted_extent_i / template_sorted_extent_i
area_ratio = environment_extent_u * environment_extent_v
             / (template_extent_u * template_extent_v)
```

部分観測では環境側が小さくなるため、小さい比率は未観測による弱い根拠として扱い、反証にはしない。
一方、対応済みの環境平面がテンプレートより極端に大きい場合は、物体外の面を取り込んだ仮説として
強く減点し、上限超過時は候補を反証する。

初期値は`basket`の実測extent比0.645から1.206を踏まえ、次の設定から検証を始める。

```yaml
plane_feature:
  extent_ratio:
    min_observed_ratio: 0.20
    min_full_match_ratio: 0.70
    max_full_match_ratio: 1.05
    max_overflow_ratio: 1.30
  area_ratio:
    min_observed_ratio: 0.15
    min_full_match_ratio: 0.60
    max_full_match_ratio: 1.10
    max_overflow_ratio: 1.40
```

- `min_observed_ratio`未満: 未観測扱い。対応候補の生成と反証の両方から除外
- `min_full_match_ratio`から`max_full_match_ratio`: サイズ一致として満点
- `max_full_match_ratio`から`max_overflow_ratio`: 超過量に応じた連続的な減点
- `max_overflow_ratio`超過: 平面サイズ超過による反証候補

サイズ超過を反証に用いるのは、法線・yaw・平面間関係で対応済みであり、対象GNG成分または
一致した非平面特徴へ接続し、かつ`planarity`・`residual_ratio`・支持node数が十分な平面だけとする。
テーブルや背景の無関係平面、観測されなかったテンプレート平面は反証に使わない。

### 非平面部分グラフの抽出

持ち手、フック、突起、穴周辺のような特徴的構造は、大きな平面クラスタへ十分に含まれない。
現状にはこの特徴を抽出する専用処理がないため、`nonplanar_graph_feature_extractor_node`を追加する。

抽出は部分グラフ総当たりではなく、平面clusterを本体coreとして使う線形時間処理とする。

```text
TopologicalMap + PlaneClusterArray
  -> 高信頼度平面nodeのcoreマスク
  -> 残余nodeの誘導部分グラフ
  -> 残余nodeの連結成分
  -> core接続境界のanchor群
  -> 非平面部分グラフ特徴
```

`basket`の実測では、テンプレートは平面所属247nodeと残余32node、環境は平面所属347nodeと
残余45nodeだった。環境残余は9成分で、最大成分は28node、平面coreへ21 edgeで接続する。
この成分が持ち手候補として抽出すべき対象である。

平面cluster IDは分割と重なりで変化するため、持ち手の条件を「接続平面IDが2個」としてはならない。
残余成分からcoreへ出る境界nodeを空間・グラフ距離で群化し、離れた`anchor_group`の個数と関係を使う。

### 非平面部分グラフの記述子

各残余成分を一般的な部位候補として出力する。最初から`handle`と意味ラベルを決めない。

```text
graph_feature:
  feature_id
  node_indices
  anchor_groups
  anchor_plane_ids
  anchor_chord_length_normalized
  anchor_delta_z_normalized
  graph_arc_length_normalized
  arc_to_chord_ratio
  normal_turn_statistics
  edge_length_statistics
  rho_statistics
  residual_covariance_shape
  confidence
```

持ち手候補では、離れたanchor群、anchor間のグラフ経路、弧長と弦長の比、法線変化、
局所edge長の連続性を優先する。GNG表面グラフには平面上にも閉路があるため、単純な閉路検出だけを
持ち手の条件に使わない。

遮蔽で片側anchorや弧の一部だけが見える場合は、低信頼度の部分特徴として残す。
特徴が見えないことは減点しない。

### テンプレート特徴index

テンプレート保存時または初回読込時に、平面clusterと非平面部分グラフから特徴indexを作る。
将来的にはテンプレートJSONへ`graph_features`として保存する。

```text
graph_features:
  - feature_id: handle_0
    feature_type: curved_bridge
    node_indices: [...]
    signature: {...}
  - feature_id: side_plane_0
    feature_type: plane
    plane_cluster_id: 2
    signature: {...}
```

indexは観測側の特徴からテンプレート候補を逆引きするために使う。特徴は複数のテンプレートへ
登録可能とし、識別力が低い平面単体だけで物体確定しない。

### 仮説生成と検証

照合はテンプレート全体から開始せず、観測済みの強い特徴から起動する。

```text
環境平面特徴または非平面部分グラフ特徴
  -> テンプレート特徴indexの逆引き
  -> feature_idとyaw候補を持つ部分仮説
  -> anchor近傍へ局所GNG対応を伝播
  -> 平面構造、edge構造、残差共分散で加点
  -> 観測済みの矛盾構造で反証
  -> confirmed時だけtemplate graphを召喚
```

平面が見える場合は平面から、持ち手が見える場合は持ち手から仮説を起動する。
両方が見える場合は強い確信とする。どちらか一方だけなら部分仮説として保持する。

テンプレート側の未観測部分は遮蔽または視野外として扱い、減点しない。
観測された特徴がテンプレート構造で説明できない場合だけ反証に使う。

### 反証

現在の反証は、対応済みGNG nodeへ接続する説明不能nodeの`winner_point_count`を集計して
`contradiction_point_ratio`を作る。高scoreでも反証率が上限を超えた候補は破棄し、次点yaw候補を使う。
全候補が破棄された場合は`no_hypothesis`とし、テンプレートを召喚しない。

ただし、node単体の貪欲対応だけでは余剰nodeを誤って反証として数えやすい。
反証は、平面または非平面特徴から得たseedを使い、局所グラフ対応を十分に確定した後段で適用する。

### 実装順

1. `nonplanar_graph_feature_extractor_node`の追加
2. 残余node成分、anchor群、基本記述子のpublish
3. テンプレートJSONへの`graph_features`保存
4. 平面特徴と非平面特徴の逆引きindex
5. 特徴起動のyaw候補生成
6. seedからの局所edge構造伝播照合
7. 共分散固有値と`rho`の補助ファジー評価
8. 局所対応確定後の反証評価
9. Viewerへの特徴、対応、未観測領域、反証根拠の可視化

最初の到達点は、環境側で抽出した最大残余成分をテンプレートの`handle_0`候補として
逆引きし、平面由来yawと整合するかをscore化するところとする。
