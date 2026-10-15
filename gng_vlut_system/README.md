# Multi-Layer Geometric GNG Project

このプロジェクトは、Growing Neural Gas (GNG) アルゴリズムを用いたロボットアームの軌道計画および自己干渉回避システムです。
角度空間と座標空間のデュアルスペース学習、および幾何学的な自己干渉判定を特徴としています。

## 詳細仕様

- [docs/TECHNICAL_SPEC.md](docs/TECHNICAL_SPEC.md)
- [docs/RELEASE_NOTE_TEMPLATE.md](docs/RELEASE_NOTE_TEMPLATE.md)
- [docs/releases/](docs/releases/)

## 依存ライブラリ (Dependencies)

- **Eigen3**: 解析計算、ベクトル・行列演算に使用。
- **urdfdom** / **urdfdom_headers**: URDFロボットモデルのパースに使用。
- **tinyxml2**: XMLのパースに使用（urdfdomで使用）。
- **console_bridge**: ロギングユーティリティ。
- **ROS 2 Humble**: ROS 2関連のノード、ビルドシステム、可視化に使用。

---

# GNG / VLUT オフライン処理 & ROS 2 ガイド

このドキュメントは、GNG (自己増殖型ニューラルガス) と VLUT (Voxel Look-Up Table) のオフライン学習から、ROS 2環境での実行、可視化までをまとめた総合ガイドです。

## 1. ビルド

まず、`gng_vlut_system` パッケージをビルドします。

```bash
# ワークスペースのルートに移動
cd ~/arm_avoidance 

# ROS 2 環境をセットアップ
source /opt/ros/humble/setup.bash

# ビルド
colcon build --symlink-install --packages-select gng_vlut_system

# ビルド後の環境をセットアップ
source install/setup.bash
```

## 2. オフライン学習

URDFからのGNG・ノードVLUT構築。既定モデルは`topo_dual_arm_max_long`。

### 基本的な学習コマンド

```bash
ros2 launch gng_vlut_system offline_urdf_trainer.launch.py experiment_id:=my_robot_v1
```

設定: `params_file`で機体YAMLの絶対パス、`experiment_id`で保存ディレクトリ名、`gng_data_directory`で保存ルート。格子幅はYAMLの`gng.vlut_resolution`［m］。

### VLUTのみ再構築

同じ保存先の既存GNGを使用した、VLUTだけの再構築:

```bash
ros2 launch gng_vlut_system offline_urdf_trainer.launch.py experiment_id:=my_robot_v1 vlut_only:=true
```

### 出力ファイル

保存先: `gng.data_directory/<experiment_id>/`。独立腕設定では各プロファイルのサブディレクトリ。

- `gng.bin`: 学習済みのGNGマップ。
- `vlut.bin`: 生成されたVLUT (Voxel Look-Up Table)。

C++の`DenseSpatialIndex`では、セルごとの開始位置・件数を、終端を含む32bit累積オフセット配列として保持。検索は隣接オフセットの直接参照で、関係配列の展開なし。読込用の中間レコードは、復号済みセル番号とノードID・距離・リンクIDの16バイト。各IDの幅・距離精度・`vlut.bin`の形式は従来どおりで、データ再生成は不要。使用するC++ノード・ツールの再ビルドが必要。Python側の独立したVLUT索引はこの変更の対象外。

`gng.bin`の保存・読込みはv10のみ。可操作性の2値はdouble全体を保存し、独立した状態フラグ5項目は1バイトへ格納。v9比では精度修正によってノード当たり4バイト増加。学習側・読込み側・起動前検査・診断スクリプトは同じ形式を使用。形式不一致は起動時に拒否し、通常処理での自動変換なし。

実行時のノード指標は、計算用`ManipulabilityEllipsoid`と常設用`manipulability_sample`を分離。常設対象は計算済みの可操作度・条件数・最小特異値・特異値3個・主軸行列・有効状態。方向評価は保持した主軸の直接利用で、行列復元・固有値分解なし。中心・体積・XYZ別の派生値はノードごとの重複保持なし。動的指標の常設対象は、現在の評価処理が参照するスカラーと有効状態のみ。ROSメッセージとGNG v10の保存項目・精度は従来どおり。C++の`GNG::Status`配置変更に伴い、関連コードの再ビルドが必要。

把持候補と経路の位置・回転可操作度は、有効な事前計算値の直接参照。無効な場合だけ、指定運動連鎖からのヤコビアン・可操作度の再計算。

旧形式v5〜v9は、ワークスペースのルートで一度だけ移行してください。退避先には未作成のディレクトリを指定。角度・座標・接続・状態フラグの照合と元データ退避後の原子的なファイル置換。部分保存された可能性のある可操作性値は無効化され、実行側の可操作性計算または再学習による再評価が必要です。独立腕の共通Viewer launchではモデルの`root_link`・`eef_link`に基づく起動時の再計算が有効。VLUTの再生成は不要。

```bash
python3 gng_vlut_system/tools/migrate_gng_v10.py gng_vlut_system/gng_results \
  --backup-dir log/artifacts/gng_v10_import/originals
```

コンテナ所有の学習データは同じコマンドをコンテナ内の`/ros2_ws/src`で実行。移行後は`gng_vlut_system`を再ビルドし、利用中の関連ノードを再起動してください。新形式の既存ファイルは内容を保持。

### 任意の辺掃引バックエンド

既定: `vlut_backend:=node_vlut`。従来の学習・保存形式・ノード判定の維持。追加方式は明示指定時のみ。

```bash
ros2 launch gng_vlut_system offline_urdf_trainer.launch.py \
  experiment_id:=my_robot_v1 vlut_only:=true vlut_backend:=edge_vlut
```

追加成果物: 各保存先の`edge_vlut_recipe.json`と`edge_vlut.npz`。既存の`gng.bin`・`vlut.bin`とは別保存。上記launchでは従来構築後に追加索引を生成し、追加成果物だけを更新。追加構築の失敗はlaunchエラー。

レシピから追加索引だけの構築、または格子条件の変更:

```bash
ros2 run gng_vlut_system gng_edge_vlut.py \
  --input /ros2_ws/src/gng_vlut_system/gng_results/my_robot_v1 \
  --voxel-size 0.05 --clearance-margin 0.02
```

既存追加索引の上書きは`--allow-replace`指定時のみ。URDF配置の変更は`--urdf`、計画グループの変更は`--avoidance-config`で、実行時と同じ`dual_arm_avoidance_demo`設定を指定。自動構築launchにも`avoidance_config`指定口あり。

- 構造: GNGの隣接辺ごとの線形関節軌道 → 理想外接球の掃引セル → セルから辺への疎な逆引き。全関節格子の列挙なし。
- 判定: 非交差は`clear`、サンプル球に完全包含されるセルとの接触は`blocked`、境界・未構築は`unknown`。通過判定には自己干渉・固定環境の離隔下界も必要。
- 環境登録: 同一点群スナップショット・格子条件につき1回。占有の追加・消去による接触カウンタ更新。内部離隔の細分はFKだけで、掃引セルの再登録なし。
- 予算: 既定512辺・60 s／レシピ、128サンプル／辺・細分段階、500,000逆引き関係。CLIの`--max-num-edges`・`--max-build-sec`で調整。未処理辺は通過扱いにならず、従来の区間検査へ委任。
- 条件照合: 全関節端点・形状・計画リンク・固定環境・入力セル半径・要求余裕。不一致または境界判定は従来検査。固定関節や非対象腕の姿勢変更も対象。

ROS側の選択口: `dual_arm_gng_lidar_demo.launch.py`／`dual_arm_gazebo_demo.launch.py`の`vlut_backend:=edge_vlut edge_vlut_file:=<索引の絶対パス>`、または回避設定YAML内の同名項目。既存ノード安全判定は継続し、確定衝突辺だけ探索から早期除外。`vlut_backend:=node_vlut`で従来方式へ復帰。

検証範囲: Longの小規模学習launch・旧バイナリ一致・19関節形状の7軸／14軸構築・ROSノード選択。ブラウザ独立腕計画・Harmonic入口・Gazebo物理を含む通し回避・実機は未検証または未接続。動的障害物の予測、実機追従誤差、未知領域の安全保証は対象外。初回読込み・環境更新の追加費用があるため、全条件での高速化保証なし。

### 3D占有に基づく領域ノード

選択: `vlut_backend:=region_vlut`。既存GNGを種に、占有変化に応じた姿勢領域を追加する試験バックエンド。元GNGの学習則・ノードVLUT・保存形式は変更なし。既定の構築方式は`legacy`。

```bash
ros2 launch gng_vlut_system offline_urdf_trainer.launch.py \
  experiment_id:=my_robot_v1 vlut_only:=true vlut_backend:=region_vlut \
  region_construction_method:=efficient
```

保存: 共通の`edge_vlut_recipe.json`から`region_vlut.npz`を追加。辺占有索引の生成・読込みは不要。既存レシピだけの構築:

```bash
ros2 run gng_vlut_system gng_region_vlut.py \
  --input <レシピまたは保存ディレクトリ> --construction-method efficient
```

| 構築方式 | 領域制約 | 構築方策 |
| --- | --- | --- |
| `legacy` | 連動一次制約 | 従来の成長・細分・統合 |
| `occupancy_cover` | 連動一次制約 | 占有変化による共鳴、包絡中心選択、被覆保持統合 |
| `quadratic_cover` | 連動二次制約 | 楕円体の全域包含を保持した成長・統合 |
| `efficient` | 連動一次制約 | 占有予算3候補のPareto選択、手先保護姿勢の追加 |

- 制約取得: `constraints()`。一次制約は`q=q0+Gλ, λ≥0, Σλ=1`、二次制約は`q=c+Dz, ‖z‖≤1`と関節制限の共通部分。二次形式・生成部分空間も取得可能。全対象関節の連動集合で、関節ペアだけの切出しではない表現。
- 3D包絡: 対象リンク外接球、Jacobianの一次像、全域二次剰余、局所レバー上界。高次元格子・全関節組合せの列挙なし。内部離隔の細長い領域には方向微分と全域剰余による区間下界。占有予算超過の早期棄却と不変球のセル再利用。
- 接続被覆: 接続全体を含む側の記録、大きい区間の中間領域追加。接続情報は被覆フラグ・端点・経由列のみ。ボクセル→辺の専用掃引索引なし。統合時の既存連続領域の包含保持。
- 環境更新: 同一の不変点群スナップショット・格子条件につき1回の登録、追加・消去セルによるカウンタ更新。粗索引の境界だけ元の点と連続包絡を照合。追加FK・入力点群の膨張・再ボクセル化なし。境界照合上限128点／領域、超過時は`unknown`。
- 生の領域状態: `region_states()`。`clear`は領域全体の環境・内部離隔成立、`blocked`は全姿勢共通の内側セルへの接触、その他は`unknown`。代表姿勢の衝突だけで全領域を`blocked`としない定義。
- 遷移保証用状態: `graph_states()`。生の領域状態が`clear`でも、登録済み隣接接続・元グラフの経由列に未証明の通過があれば`unknown`。この状態での安全側と隣接要確認側の通過保証。任意の非隣接ペア・外部GNG安全ラベルへの保証ではない区別。

| 状態ペア・条件 | 遷移判定 |
| --- | --- |
| 接続全体を覆う側が`clear`、他側が`clear`／`unknown` | 通過可能 |
| いずれかが`blocked` | 棄却 |
| 上記以外、または被覆・照合条件の不成立 | 従来の区間検査 |

姿勢の欠落対策:

- 必須包含: 元GNGの全姿勢と`required_poses:=<JSONの絶対パス>`／CLIの`--required-poses`で指定した全姿勢。JSON形式は全可動関節の姿勢行列、列順は保存モデルの`joint_names`、回転rad・直動m。`legacy`での保護指定は拒否。包含できない構築予算では明示エラー。
- 自動保護: `efficient`で既定8姿勢。有限の可行候補から手先の位置・向きの低被覆姿勢を選択し、近傍への接続候補を追加。既存`gng.bin`のノード追加や外部GNG探索グラフの差替えはなし。
- 未登録姿勢: `locate()`の`-1`は到達不能ではない値。`residual_constraints()`で関節範囲から明示領域を除いた残余を`unknown`として保持。`constraints_for_pose(geometry, q)`でモデル上の可行姿勢の局所制約を補完。上限32領域、生成予算0.1 s／姿勢、生成失敗時は当該姿勢の点制約。通常の`query()`での局所包絡生成なし。
- 未達範囲: 全可行姿勢の明示領域被覆、残余を含む全空間の接続証明、特殊手先位置へのIK解生成。残余の保持と局所補完だけを、これらの達成と見なさない区別。

評価・選択:

指標の定義・現行出力の限界・保存索引のノード別集計・今後の採否候補は[姿勢領域ノードの評価指標一覧](docs/designs/gng_region_evaluation_catalog.md)。候補一覧への登録と、実装・最適化目的への採用は別扱い。

- 必須条件: 種姿勢・保護姿勢の欠落0件、安全／棄却の参照矛盾0件、遷移保証違反0件。評価の対象は保存済み姿勢グラフと点群配置、参照は密なFKとサンプル間の連続下界。参照未確定区間は別計数。
- 目的: 保護姿勢を含む接続対の喪失数、全接続対の喪失数、安全参照区間の要確認数、領域数、逆引き関係数。単純な重み付き和ではなく、限定候補の非支配集合からの選択。
- 既定目標: 接続保持率98 %、保護姿勢接続保持率98 %、安全参照区間の要確認率5 %。達成候補の中で領域数・逆引き関係数を優先。達成候補なしでも最良候補を保存し、`can_meet_selection_targets=false`を出力。正常終了と目標達成の区別。
- 資源: メモリ、制約値数、構築時間、環境更新p95、従来検査を含む実照会時間、保持経路の伸長率。候補選択用配置と独立検証用配置の乱数種の分離。選択スコアだけでは汎化性能の証明にならない評価。

既定予算: 256領域、64頂点／領域、占有増分96セル／領域、構築60 s／レシピ、500,000逆引き関係。CLIの`--max-num-regions`・`--max-extra-cells-th`・`--max-build-sec`で調整。`efficient`は占有予算24／48／96セルの3候補、構築時間を3分割。評価時間は別枠。接続未被覆・索引未登録は従来検査への委任。全空間の被覆保証なし。

ROS側: 上記の回避launchで`vlut_backend:=region_vlut region_vlut_file:=<絶対パス>`、または回避設定YAML内の同名項目。環境登録・領域状態・状態ペアはスナップショット内で共有。外部のGNGノード安全ラベル・入力鮮度・保持／停止の条件は維持。

検証範囲: Longの19関節形状・14軸、直動を含む包絡、全関節二次領域の占有包含、未登録の可行6姿勢の補完、領域数上限での欠落拒否、旧方式回帰、構築launch・旧バイナリ一致・ROS切替・非同期探索。Python参照版の疎な実学習8姿勢＋保護8姿勢では、既定の選択目標は未達。領域構築・メモリ・環境更新に追加費用があるため構築方式の既定切替なし。

比較条件: Longの14軸・中立姿勢付近±0.04 rad、7種姿勢・8接続、0.025 m格子、入力点半径0.01 m、独立96配置×5回。自動手先保護なし、全方式で同一姿勢・接続。4方式×5回の成功20／20件、参照矛盾・遷移保証違反・種姿勢欠落0件。以下2段落はPython数値部品での測定、表は5回の中央値。

| 指標 | `legacy` | `efficient` |
| --- | ---: | ---: |
| 領域数 [個] | 6 | 6 |
| 安全参照区間の要確認率 [%] | 15.7 | 0.0 |
| 接続保持率 [%] | 84.1 | 100.0 |
| 768区間照会の全体時間 [ms] | 381 | 247 |
| 環境更新p95 [ms] | 0.20 | 0.60 |
| 配列メモリ [KiB] | 279 | 378 |
| 構築・候補選択時間 [s] | 3.33 | 27.26 |

中央値0 %は全試行の要確認0件ではない値。`efficient`の合算要確認率0.087 %、最小接続保持率99.7 %。疎な実学習8姿勢＋保護8姿勢・独立24配置では、既定構築の接続保持率12.3 %、保護姿勢接続保持率5.4 %、姿勢欠落0件。単一候補75 s・占有増分384セルではそれぞれ93.3 %／93.5 %、184領域。後者も選択目標未達で、局所比較の改善を全可動域へ一般化しない範囲。

理論の境界: [SPARS](https://people.cs.rutgers.edu/~kb572/pubs/DobsonBekris_Spanner.pdf)の被覆・接続性・経路品質の分離を評価の参考とする構成。SPARSの確率的完全性・近似最適性の定理の継承なし。[IRIS-NP](https://arxiv.org/abs/2303.14737)の領域構築と全空間被覆も別問題。[ARMTD](https://arxiv.org/abs/2002.01591)の動的到達集合・制動込み保証の実装なし。対象は理想的な球形状と静的環境。未知区間の従来サンプル検査は連続安全証明ではなく、MPC・GUI領域表示・物理を含む通し回避・実機・動的障害物予測は未接続または未検証。

#### C++数値部品

変位上界の係数計算、点・支持平面包絡の判定、領域索引の差分カウンタ集計は`src/core/collision/region_kernel.cpp`。C ABIの共有ライブラリ`libgng_region_kernel.so`、Python／ROS SDKへのビルド依存なし。数値境界だけPython参照判定への委任。既存の起動引数・保存形式・入力点群登録回数の変更なし。

差分集計: 登録済みCSRの直接走査による関係配列の展開省略。追加APIのない旧ライブラリでは当該処理だけPython経路。新APIの利用はライブラリの再ビルド後。更新処理の短縮と、未被覆区間の従来検査を含む照会全体の短縮は別評価。

- 配置: 通常パッケージビルドに同梱、インストール先`lib/gng_vlut_system`。ROS非依存の単独ビルドはCMakeの`enable_region_kernel_only=ON`。現在のDocker環境にも配置済み。
- 選択: `GNG_REGION_KERNEL=auto`が既定、配置済みならC++、未配置ならPython。`cpp`はC++必須、`python`は参照版固定。C++の明示指定・ABI不一致・計算失敗時の黙示的な代替なし。
- 検証用パス: `GNG_REGION_NATIVE_LIBRARY=<共有ライブラリの絶対パス>`。配置・環境変更の反映はプロセス再起動後。保存metadataの`kernel_backend`は構築時の部品、照会時の部品は現在の実行環境に従う区別。
- 比較条件: `occupancy_cover`、同一仕事量、各5回中央値。局所は上記14軸・7姿勢／8接続、広域は7軸・8種姿勢＋保護8姿勢／41接続、0.05 m格子、細分上限2。時間切れによる仕事量の差なし。

| 対象 | 指標 | Python | C++ |
| --- | --- | ---: | ---: |
| 局所 | 構築 [s] | 5.24 | 3.08 |
| 広域 | 構築 [s] | 42.12 | 18.33 |
| 広域 | 環境更新p95 [ms] | 5.31 | 3.47 |
| 広域 | 984区間照会の全体時間 [s] | 6.22 | 6.45 |

C++化単独での照会全体の短縮は未確認。上界・占有・境界の2,258チェックで不一致0件、構築比較20／20件成功。対応10組・300配列で、整数／占有配列は完全一致、浮動小数配列は絶対許容誤差1e-11・相対許容誤差1e-12内。ROS自動読込み・spawn探索・指令0件・停止後ノード0件も確認。被覆・接続保持の方式改善は別課題、C++版の既定時間予算による広域目標達成は未検証。

#### 内部離隔証明による検査分離

非`legacy`の追加索引には、元姿勢ペア間の線形関節区間に対する連続内部離隔下界も保存。全関節端点・球形状・運動連鎖・固定環境・要求離隔の一致時のみ、オンラインの自己干渉・床・机の再検査を省略。外部点群検査は継続、未証明・不一致は従来検査。事前証明予算は構築予算内の10 %、上限5 s。証明情報のない旧索引は従来動作のまま、追加索引の再構築後に有効。

構築方策は`coupled`が既定。占有領域の成長を内部離隔と分離する試験方策は、launchの`region_internal_policy:=separate`、または以下のCLI指定。`legacy`との組合せは拒否。

```bash
ros2 run gng_vlut_system gng_region_vlut.py \
  --input <レシピまたは保存ディレクトリ> \
  --construction-method occupancy_cover --internal-policy separate
```

- 検査API: `query_checks()`の`can_pass`と`can_skip_internal`。外部索引の未被覆区間でも、内部証明だけの再利用が可能。中間自己衝突・床接触の実姿勢証拠は棄却、未証明は従来検査。
- 領域API: 外部点群だけの`external_region_states()`と、従来互換の`region_states()`・`graph_states()`。分離領域には内部不可行な姿勢が含まれ得るため、外部`clear`だけで全姿勢の通過許可は不可。`constraints()`に内部離隔下界`min_internal`を併記。

比較条件: Longの19関節形状・7対象軸、8種姿勢＋保護8姿勢、41区間、0.05 m格子、独立24配置×5回、各配置1〜5点。C++部品を全条件で使用、索引の読込み・環境更新・未被覆区間の従来検査を含む984区間照会。参照は同一の結合構築索引で内部証明の再利用だけ無効。表は5回の中央値。

| 指標 | 参照 | 結合構築＋証明 | 分離構築＋証明 |
| --- | ---: | ---: | ---: |
| 984区間照会の全体時間 [s] | 6.01 | 2.21 | 2.22 |
| 区間照会p95 [ms] | 14.86 | 5.11 | 5.12 |
| オンライン内部検査姿勢数 [個] | 18,665 | 0 | 0 |
| 領域数 [個] | 105 | 105 | 104 |

成功15／15試行、9,840回答の比較不一致0件、参照矛盾・遷移保証違反・種姿勢／保護姿勢欠落0件。区間証明41／41件、事前計算0.42／0.43 sは各1回の測定。短縮の主因は内部再検査の省略、分離自体の領域削減は小幅で接続保持の改善なし。安全参照区間の要確認率は約56 %、索引の接続保持率は約35 %のまま。全可行姿勢の明示被覆・実機の連続安全保証は未達。多数点群・実機誤差・動的障害物への性能一般化は対象外。

#### 実URDFからの角度箱構築

既存GNGとは別の追加方式。CLI既定の`hierarchical_cover`は親領域と内部葉を持つ`urdf_vrm_hierarchy_v1`、旧単層方式は`--construction-method urdf_boxes`で維持。内部葉は各対象関節の一次制約`q_min ≤ q ≤ q_max`。URDFのFK・外接球・一次像＋二次剰余による3D占有包絡、球表面積×Jacobianによる分割順位。配布版の親・葉構造の移植であり、ボクセル重み付きL∞被覆の逐語移植ではない区別。

ワークスペース直下での構築・表示。既定対象はLongの左腕7軸、全19関節の形状とmimicを保持、他の独立関節は制限内の0位置で固定。

```bash
python3 gng_vlut_system/scripts/urdf_vrm.py
python3 tools/gng_region_viewer/viewer.py gng_results/topo_dual_arm_max_long/urdf_hierarchy_left_arm.npz
```

右腕は`--group right_arm`、標準Maxは`--params-file topo_dual_arm_max.yaml`。別の実URDFと対象関節の指定:

```bash
python3 gng_vlut_system/scripts/urdf_vrm.py \
  --urdf /path/to/robot.urdf --joint-names joint1 joint2 joint3 \
  --output /path/to/new_model.npz
python3 tools/gng_region_viewer/viewer.py /path/to/new_model.npz
```

- 入力: 展開済みURDF。固定・回転・連続回転・直動関節、独立関節を参照するmimic、STL／box／sphere／cylinder。相対／絶対パス・`file://`・`package://`のローカルメッシュ。未対応形状・関節・資源の欠落は明示エラー、黙示的な省略なし。
- 単独構築の自己干渉: 同一剛体・隣接剛体・設定での明示除外だけを除くペア。初期姿勢の重複による自動除外なし。床・机の暗黙追加なし、床は`--floor-z`指定時だけ。
- 構築予算: 既定128親・親内16葉・30 s・葉展開換算500,000関係。`--max-num-regions`・`--max-num-leaves-per-parent`・`--max-build-sec`・`--max-num-relations`で変更。親×内部葉の上限2,048葉、内部葉上限の指定範囲2〜16個。床境界・予算超過の葉も分割木に保持、空環境だけによる安全への昇格なし。保存先の既存ファイルは拒否、明示上書きは`--allow-replace`。
- 入力登録: 既存`update_cloud()`の1回。ボクセル→親ID・葉ビットの圧縮CSR、C++差分カウンタ、親の粗交差→必要な葉の連続区間交差。C++共有部品の更新はパッケージ再ビルド後に有効、旧共有ライブラリ時はPython互換経路。
- 被覆の範囲: 対象関節の有限可動域の分割保持と、全域の索引化・可行姿勢の安全証明は別。連続関節の窓は`[-π, π]`、固定関節値の変更・窓外は要詳細検査。全可動域の効率的な索引化は未達。
- 判定: 外接球の重複だけによる実メッシュ衝突の確定なし。内側索引は空集合、未知は詳細検査へ委任。線形関節区間と箱の連続交差で照会、隣接面の接続候補と通過保証は別。自己干渉の無視による高速化なし。
- 状態・制約API: `parent_states()`・`parent_constraints(parent_idx)`、選択葉の`linear_constraints(leaf_idx)`。後者はQP向けの行列・上下限・三値状態。親が要確認でも安全な内部葉は保持。安全葉の和集合を単一の凸包へ置換しない条件。
- 接続状況: 回避launchの専用選択肢`hierarchy_vlut`と、`region_vlut.load()`による旧形式・階層形式の自動識別。保存条件に対応したViewerの親別内部葉表示。既存GNG・二次方式・回避launchの既定動作は維持。MPC制御器・Gazebo全体・実機の動作は未検証、速度・制動・実機安全の保証なし。

##### 回避バックエンドの差し替え

選択: `vlut_backend:=hierarchy_vlut hierarchy_vlut_file:=<npz絶対パス>`。差し替え対象は区間衝突照会。姿勢グラフ・ノード安全ラベル・経路探索・入力鮮度・保持／停止条件は既存基盤の継続。`node_vlut`・`edge_vlut`・`region_vlut`も維持、既定は`node_vlut`。

パッケージ再ビルド・setup読込み後のLong左腕の例。構築launchの既定機種はLong、回避launchへの機種指定も同一設定。

```bash
gng_share="$(ros2 pkg prefix --share gng_vlut_system)"
ros2 launch gng_vlut_system offline_urdf_trainer.launch.py \
  vlut_backend:=hierarchy_vlut hierarchy_vlut_file:=/tmp/long_hierarchy.npz

ros2 launch gng_vlut_system dual_arm_gng_lidar_demo.launch.py \
  params_file:="$gng_share/config/topo_dual_arm_max_long.yaml" \
  vlut_backend:=hierarchy_vlut hierarchy_vlut_file:=/tmp/long_hierarchy.npz
```

- 構築launch: GNG再学習なしの独立索引生成。既存`gng.bin`・`vlut.bin`の変更なし。回避実行には対象機種の既存グラフ・ノードVLUTが別途必要。保存先の既存ファイルは拒否。
- 構築条件: `avoidance_config`は既定で`dual_arm_gng_lidar_demo.yaml`。CLI直接構築では`--avoidance-config`で指定。実行系と同じURDF・監視リンク・床／机・既存自己干渉規則の使用。単独構築の厳格ペアとの区別、`--floor-z`との併用不可。標準Maxは構築・実行の双方に同じ`params_file`の指定。
- 起動時検査: 専用選択では未指定・欠落・形式違い・形状署名不一致をエラー。索引葉0件は警告。従来の`region_vlut`経由の自動識別・不一致時の詳細検査への委任は維持。
- 対象範囲: 既定は左腕7軸、他の関節は構築時の固定値。`hierarchy_group:=right_arm`で右腕用構築。対象外姿勢・未索引葉・離隔証明不足は従来の詳細検査。両腕同時運動の全域証明なし。
- 代表候補の棄却: `gng_path_search.can_bridge(..., target_node_id=ID)`で、衝突ラベル`2`かつ終点がその代表姿勢に完全一致する場合は詳細照会前に棄却。別姿勢・危険ラベル`3`・ID未指定時は既存照会の継続。代表姿勢の衝突から領域全体の衝突への伝播なし。
- 検証: 切替・旧形式互換・詳細検査への委任・ROS launch引数の12／12試行、600区間照会の比較不一致0件。Max／Longの標準予算構築・再読込み・Viewer出力も各1回成功。[再現設定・結果](../benchmarks/hierarchy_backend/results.json)。実URDFの索引化範囲は限定的、性能優越・Gazebo全体・実機安全の検証とは別。

単層方式の既存検証: Long左右7軸・標準Max左7軸・汎用4軸の各2回、5,592チェック不一致0件。固定変換／mimicのFK照合、有限標本での包絡・占有セル・内部下界、全対象窓内の領域所属、未索引の要確認、保存再読込み、旧索引読込み。階層方式の検証は[評価正本](docs/designs/gng_region_evaluation_catalog.md#配布更新版vrm_cpp-1の階層検証)。有限標本の一致と全姿勢の実機安全保証は別。

改善判定: [固定基準・三方式の同条件比較](docs/designs/gng_region_evaluation_catalog.md#0-改善の判定基準と同条件比較)は旧単層箱の結果。階層方式の優越性へ読み替えない条件。一次を基本方針、二次は比較・選択肢として維持。現状は既存GNG-VLUTの改善版としての既定採用なし。既存方式も広域姿勢被覆の目標は未達。

自己干渉の制限: CLI既定の厳格ペアでは、右腕固定時にグリッパーと手首カバーの外接球が重なり、当該固定条件の内部可行域を得られない状態。実メッシュの干渉・球の過大近似・隣接機構の除外不足の切分けは未完了。上記の方式比較は既存自己干渉規則へ揃えた別条件、既定の厳格規則での回避成功を示す結果ではない区別。

#### 保存領域の3D Viewer

ワークスペース直下で、保存済み`region_vlut.npz`を指定した読取り専用の起動:

```bash
python3 tools/gng_region_viewer/viewer.py <region_vlut.npz>
```

URL: `http://127.0.0.1:8785`。終了: 実行端末で`Ctrl+C`。別ポートは`--port`。既存のNumPy・SciPyと同梱Three.jsを使用、npmビルド・ROS・実機接続は不要。このworkspaceの比較用索引は`log/artifacts/gng_internal_certificate/separate.npz`。

- 選択: 領域ノード、隣接／その他ノードとの比較。橙は選択側だけ、灰は共通、緑は比較側だけ。元GNGの点ノードIDとは別の領域ID。
- ボクセル: 全域占有包絡、全ノードに対する独自占有、照会用の余裕込み、基準姿勢、全姿勢共通の内側。保存セルをそのまま使用、表示切替・姿勢例再生での再ボクセル化なし。セル体積和は格子幅の3乗による集計。
- 関節領域: 選択2軸への投影、関節ごとの周辺範囲、元の制約。一次域は全頂点の凸包投影、二次域は関節制限で切る前の楕円体の投影外包。URDF箱方式は上下限による矩形投影・索引状態も表示。いずれも他関節の固定断面とは別。
- 姿勢例: 基準姿勢と領域内の有限例を球近似で表示・再生。既定最大12例／領域、`--max-num-pose-samples`で2〜32例。例だけのFKは起動時、ブラウザでは保存済み中心座標の切替。全領域の占有は例の和集合ではなく保存包絡。
- 条件: 従来形式の既定URDFはLong。別モデルは`--urdf`、構築時の計画リンク設定は`--avoidance-config`。URDF箱方式は保存済みURDF・計画リンク・固定環境を使用、URDF移設時は`--urdf`。形状署名の不一致は拒否。`gng.bin`・`vlut.bin`単体の表示は対象外。
- JSON保存: `--export-json <保存先.json>`で配信せず終了。既存ファイルの上書きは`--allow-replace`指定時のみ。ViewerのJSON読込みボタンで別データへ切替可能、元索引の書換えなし。

描画は操作時だけ、姿勢例再生中もボクセル再生成なし。内側セル0と索引未登録は別表示。データ上限32 MiB、単一表示上限50,000セル、超過時は明示エラー。localhostの固定配信先だけを公開、URDF・ワークスペースの任意ファイル配信や操作APIはなし。実機・衝突許可・動的安全保証には未接続。

検証: 一次104領域・二次20領域、計756姿勢例の領域所属・保存占有一致・元索引非破壊。実Chromeの表示・比較・入力検証・再生・描画休止23項目、例外0件。検証用HTTPサーバ・Chromeは終了済み。

### 補助ツール (任意)

生成されたGNGマップを整理・検証するための補助ツールが用意されています。

- **島の除去 (`offline_gng_main6_island_pruning`)**: GNGグラフが分離した「島」を持っている場合に、最大の連結成分のみを残します。
- **データクリーンアップ (`offline_gng_main5_clean`)**: 非アクティブなノードを完全に削除し、ファイルを軽量化します。
- **データ一貫性チェック (`inspect_gng_data`)**: GNGマップ内のデータに矛盾がないか検証します。
- **特徴量再計算 (`offline_gng_status_updater`)**: GNGマップに可操作性などの特徴量を再計算・上書きします。

## 3. ROS 2での実行

### 基本的な可視化

RVizでロボットモデルとGNGの安全状態を可視化します。

```bash
ros2 launch gng_vlut_system visualize_topoarm_rviz.launch.py
```

### 安全監視の有効化

GNG/VLUTによる安全監視を有効にして、RVizで確認する場合。

```bash
ros2 launch gng_vlut_system visualize_topoarm_rviz.launch.py \
  enable_safety_monitor:=true \
```

### 実機/Gazeboとの連携

- **実機**の関節角度を使用する場合:
  ```bash
  ros2 launch gng_vlut_system visualize_topoarm_rviz.launch.py joint_state_source:=real
  ```
- **Gazebo**でシミュレーションを行う場合:
  ```bash
  # Gazeboサーバーとロボットを起動
  ros2 launch gng_vlut_system robot_gazebo_sim.launch.py robot_name:=ToPoDualArm

  # 別のターミナルでRVizを起動
  ros2 launch gng_vlut_system visualize_topoarm_rviz.launch.py
  ```
  Docker内でGUIが不要なら `gui:=false` にできます。

### FuzzBotの移動状態GNG

FuzzBotの初期位置・前方を原点とする状態 `(x, y, 相対yaw, v, omega)` を学習し、位置・前方向・速度をViewerへ配信します。設定は[config/fuzzbot.yaml](config/fuzzbot.yaml)の `mobile_gng` と `motion_graph`。既定は6万状態・1,000ノード・20万反復、1軌跡は最大16区間×0.5秒です。平坦床・差動二輪・横滑りなしの運動モデルで、車輪速度と加減速制限を適用します。

```bash
# 初回学習・保存済みモデルの読込・FuzzBot本体の同時表示
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py params_file:=fuzzbot.yaml

# 設定変更後の再学習専用
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py params_file:=fuzzbot.yaml \
  enable_rebuild:=true enable_build_only:=true

# 相対yaw −15～15°の状態表示
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py params_file:=fuzzbot.yaml \
  min_display_yaw_deg:=-15.0 max_display_yaw_deg:=15.0
```

ViewerまたはSimulatorの「ROS Scene Layers」で `/fuzzbot/state_gng/map` を選択し、対象レイヤーの `Normals` を有効にします。このトピックのnormalは `(cos yaw, sin yaw, 0)` の車体前方で、環境表面の法線とは異なる属性です。速度は同じ向きにvを掛けたモデル値で、GNG状態領域のcluster velocityへ配信します。`state_gng/data` は5次元状態・単位・学習属性、`state_gng/node_features` は代表姿勢です。

`gng_viewer_bridge.launch.py`は機体YAMLの`mobile_gng`設定に応じて状態GNGを同時起動します。`enable_mobile_gng:=false`で本体表示のみ。双腕設定では既定OFF。

モデルは `gng_results/fuzzbot/state_gng.json`。`model_path` で変更可能です。yawはcos・sinへ埋め込み、学習の距離は位置・向き・速度・角速度の各scaleで換算します。学習中心に近い重複なしの実サンプルを代表状態として保存します。GNG学習は共通C++エンジンと6次元の厳密近傍索引を使用します。

`origin_x`、`origin_y`、`origin_yaw_deg` は表示原点の配置のみで、学習済み状態を変更しません。yaw選択は相対角で、`min_display_yaw_deg` が `max_display_yaw_deg` より大きい場合は±180°をまたぐ区間です。`min_display_speed`／`max_display_speed`、`min_display_angular_speed`／`max_display_angular_speed` で状態を選択可能。これらの表示範囲・原点は `/fuzzbot/mobile_gng` のROSパラメータでも変更可能です。選択後の辺は表示ノード配列へ参照を付け替えます。

ROSなしの確認用HTMLは次のコマンドで生成できます。

```bash
ros2 run gng_vlut_system mobile_gng_preview.py \
  /ros2_ws/src/gng_vlut_system/gng_results/fuzzbot/state_gng.json \
  /ros2_ws/src/log/artifacts/fuzzbot_state_gng/preview.html
```

学習済み辺は状態空間のGNG近傍で、走行可能な制御入力を保証する辺ではありません。車体・掃引占有のVLUT、環境衝突判定、odomへの自動追従、経路探索と実機走行は未接続です。既存の `mobile_state_graph.launch.py` が生成する運動制約付き遷移グラフは別データとして保持します。

### 双腕学習・Viewerの既定モデル

モデル指定を省略した場合は`topo_dual_arm_max_long`を使用します。
次の両launchの`params_file`既定値は`config/topo_dual_arm_max_long.yaml`です。

```bash
# 双腕GNGの学習
ros2 launch gng_vlut_system offline_urdf_trainer_dual.launch.py

# 保存済みGNGのViewer配信
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py
```

Long設定のViewer起動は、既存Dynamixel入力から`/leader/joint_states`と`/follower/joint_states`の変換・配信も担当します。
入力設定は[config/dynamixel_joint_state_input.yaml](config/dynamixel_joint_state_input.yaml)、実機の追従設定は[config/dynamixel_leader_follower.yaml](config/dynamixel_leader_follower.yaml)。
USBドライバと実機駆動は別起動です。角度配信だけなら追従制御launchは不要です。

| 起動 | 担当 |
| --- | --- |
| `gng_viewer_bridge.launch.py` | GNG・ロボット表示、共通の関節実測トピック |
| `dynamixel_joint_state_input.launch.py` | Viewerなしの場合の共通実測入力のみ |
| `dynamixel_leader_follower.launch.py` | 共通リーダー実測の購読、実機監視・追従・停止操作 |
| `dynamixel_handler`／読取り専用reader | USB通信 |

既存入力配信の再利用時は`enable_dynamixel_joint_state_input:=false`。
GNGとロボットTFを実機フォロワーの姿勢に合わせる場合は、表示元も明示します。

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=topo_dual_arm_max_long.yaml \
  joint_control_backend:=external state_topic:=/follower/joint_states
```

s（Simulator）・r（実機リーダー）・f（実機フォロワー）の接続管理は、共通launchとブラウザで選択できます。

```bash
ros2 launch gng_vlut_system robot_follow.launch.py
```

既定モデルはLong、初期構成は手動操作、実機出力許可はOFFです。SimulatorサーバーとROS／MuJoCoブリッジは既存サービスを再利用し、未起動分だけ起動します。USBドライバの起動は含みません。
ブラウザの「ROS2連携 → s・r・fの接続構成」で、描画追従・力学追従・実機への追従元を選択します。起動時の指定例は `profile:=r_display`。
接続構成は[config/robot_follow.yaml](config/robot_follow.yaml)に集約。入力変換・経路管理・既存実機監視が共通launchの担当です。Viewerを含める場合は `enable_viewer:=true`。既存Viewer等が共通実測を配信中なら `enable_joint_state_input:=false` を指定し、同じ入力の重複起動を避けます。
設定・停止条件・実機有効化は[共通追従構成](docs/dynamixel_sim_control.md#srfの共通追従構成)を参照してください。

`state_topic`の実測表示では初期ゼロ姿勢・仮想制御の配信を無効化します。
既定の仮想Viewer操作と、実測角度のトピック配信は独立した経路です。

学習結果の保存先・Viewerの読込先は、このYAMLの`gng.data_directory`と
`gng.experiment_id`に従います。左右別モデルの保存先は
`gng_results/topo_dual_arm_max_long_independent/`です。
別モデルを使う場合は`params_file:=...`で対象の設定ファイルを指定します。

配信周期`publish_hz`の優先順は明示launch引数 → 機体YAML → 30 Hz。ViewerとGNGで共通、不正値は起動時に拒否します。
旧`gng_vlut_runtime.launch.py`は現行Viewerへの外部実測表示用の互換入口です。旧`dir`／`id`と`data_directory`／`experiment_id`は利用可能ですが、安全監視・`tag`／`mode`／`safety_margin`・初回姿勢配信の指定は対応不可として拒否します。
単腕の旧後処理launch（`offline_gng_main5_clean`、`offline_gng_main6_island_pruning`、`offline_gng_status_updater`）は対象データ用の`params_file`を明示指定してください。


`gng.enable_independent_arms: true`では左右各7関節を別々に学習します。
上限は片腕10,000ノード、初期240万回・追加10万回の配置更新が片腕ごとの設定です。
既定の`gng.enable_batched_collision_filter: true`では配置確定後に全ノードを検査し、
学習で得られた関節空間の候補辺から、各ノードの近傍6候補と最小全域森を選びます。
選んだ辺だけを最大関節差0.025 radの間隔で検査し、非干渉の接続をTCP層にも共有します。
TCP位置だけから別の辺を追加する処理はなく、この方式では`coord_edge_iterations`は未使用です。
`gng.collision_worker_num`は既定16、`gng.num_local_neighbors`は既定6です。
近傍・全域森の選択外となった候補辺は、衝突検査前に省略します。
全ての非干渉辺を保存する設定ではなく、検査対象を局所的な接続へ絞る設定です。
`omitted_unchecked_edges`が未検査の省略数、`colliding_edges`が検査で干渉した辺数です。
`mean_num_neighbors`は無向辺数の2倍をノード数で割った平均次数です。
workerは不変な衝突形状を共有し、姿勢・探索キャッシュ・FK出力をそれぞれ保持します。
干渉除去後に分かれたグラフは無理に接続せず、成分数を`[SparseFilter] Result`へ出力します。
干渉ノードを除去するため、保存されるノード数は上限を下回る場合があります。
各ノードが持つ姿勢はその腕の7関節角1組です。左右の全組合せを事前生成する構成ではありません。

保存形式は次のとおりです。

- `independent_arms.json`: 左右モデルの一覧。両方の保存成功後に生成。
- `left_arm/`・`right_arm/`: 各腕の`gng.bin`、`vlut.bin`、`model.json`。
- `model.json`: 関節名・順番、URDF、固定関節条件、ノード数、ボクセル幅。

片腕の学習では反対腕の可動部分だけを衝突形状から外し、対象腕・胴体・固定外装を検査します。
VLUTには対象腕の可動リンクと、その先の固定外装・指を収録します。
腰・首・グリッパーは0の条件です。左右それぞれ安全なノードでも、組合せが安全とは限りません。

Viewer用launchは`Tmap_left_arm`・`Tmap_right_arm`と、
`/<robot_name>/check_arm_pair`サービスを起動します。
`left_arm/topological_node_features`・`right_arm/topological_node_features`の`weight_angle`は各7要素です。
ノードIDは各腕のグラフ内でのみ有効です。環境ROIを有効にした場合の既定範囲は、
左右VLUTの占有範囲を合わせたBBoxです。`min_reachability_*`・`max_reachability_*`の明示設定を優先します。

```bash
ros2 service call /topo_dual_arm_max_long/check_arm_pair \
  gng_control_msgs/srv/CheckArmPair '{left_node_id: 0, right_node_id: 0}'
```

IDは配信グラフから選択します。`start_state`なしは終点のみの検査です。
現在姿勢を`start_state.name`・`start_state.position`に渡すと、左右14関節すべてを必須として、
開始点・終点・途中の全身自己干渉を最大関節差0.025 radの間隔で検査します。
連続時間の非干渉保証ではなく、関節角の線形補間に沿った離散検査です。
指定した固定関節が0以外の場合は拒否します。省略した固定関節は保存時の0が前提です。
`is_valid && is_collision_free`を採用条件とし、`joint_state`で組合せ後の14関節を取得します。
環境障害物・実機への送信・既存の制御経路への自動介入はこのサービスの対象外です。


従来方式（`enable_batched_collision_filter:=false`）では、追加学習中にも衝突回避を行い、
TCP辺を別に構築します。この方式の`gng_params.enable_static_collision_cache: true`は、
同一の関節角を持つ非干渉ノードと、両端の姿勢が変わっていない検査済みの辺を、
複数回の`strictFilter()`間で再利用する設定です。量子化や許容差による一致判定はなく、
姿勢変更・未検査の辺には従来と同じ検査を適用します。
メッシュ・ボクセルの判定精度と、辺の補間間隔（最大関節差0.025 rad）は従来どおりです。
初回の検査や、追加学習中に変わる姿勢の衝突判定は引き続き必要です。
比較時は次の引数でフィルタ結果の再利用を無効化できます。

```bash
ros2 launch gng_vlut_system offline_urdf_trainer_dual.launch.py \
  enable_batched_collision_filter:=false enable_static_collision_cache:=false
```

この再利用は、ロボット形状・固定姿勢・運動連鎖・除外規則・環境が不変のオフライン学習用です。
汎用GNGクラスでは既定オフです。APIで有効化した後にこれらの条件を変える場合は、
`invalidate_collision_cache()`の呼出しが必要です。チェッカー差替え・パラメータ設定・
モデル読込みでは自動的に破棄され、キャッシュ自体はモデルに保存されません。

現在の`use_voxel_collision: true`は、全身のメッシュ表面とボクセルによる内部干渉検査の選択です。
ボクセルはメッシュに完全に内包された形状の検出にも使用します。辺の検査にも同じ判定を使用します。

衝突検査では、FCLのメッシュ探索境界をリンクペアごとに再利用します。
直前の非交差検査で得た範囲から探索を再開し、各範囲を現在の姿勢で再検査します。
元の三角形・包絡判定・内部判定・関節補間間隔を維持します。
保持量の上限と定期的な再構築があり、衝突検出時の未完了範囲は再利用しません。
形状の差替えや探索順の反転では破棄します。対象は幾何不変のFCL OBBRSSメッシュのbool判定です。
他の形状と接触点取得は通常のFCL経路です。この最適化はフィルタ結果の再利用とは独立して有効です。

### ToPoFuzzy-Viewerへのブリッジ

学習したGNGマップやアームの姿勢をToPoFuzzy-Viewerに送信します。

既定のロボット名・設定は `topo_dual_arm_max_long` です。次の指定はファイル名だけでも利用でき、
`params_file` を省略した場合も同じ設定です。

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=topo_dual_arm_max_long.yaml
```

左右別モデルは`dir`・`id`で出力ディレクトリを指定して表示します。
`gng_results/topo_dual_arm_max_long_independent/independent_arms.json`から左右のファイルを読み込みます。
従来の14関節モデルを表示する場合は`enable_independent_arms:=false`と、
`gng_model_path`・`vlut_path`を指定します。以前の検査済みモデルは
`log/artifacts/gng_self_collision_fix_20261001/long/model/`に保持しています。
名前・モデル・設定の切替には、起動中のブリッジの終了と新しい設定での再起動が必要です。

別モデルのToPoDualArmは次の指定です。左右グリッパーの体積確認用トピック
（`L_grip_V_Tmap`など、左右の`V`・`minV`・`baseV`・`sweptV`）は
`ToPoDualArm.yaml`で既定オフです。

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
```

体積を可視化して確認する場合は、同じコマンドに
`enable_gripper_volume_graph:=true`を追加します。
`enable_gripper_volume_graph:=false`で発行ノードの起動を無効化できます。
引数を省略した場合は、YAMLの`gripper_volume_graph.enabled`に従います。
切替はlaunchの再起動時に反映されます。

簡略版L0は`/ToPoDualArm/Tmap_vis_L0`です。`visualization_gng.enabled: true`と
現行形式の`vis_gng_L0.bin`があれば、上記ブリッジの起動時に元の`Tmap_static`と同時配信されます。
モデルを再生成した場合はブリッジの再起動が必要です。

L0のノード色は、集約元の安全（緑）・危険（黄）・衝突／使用不可（赤）の件数割合で表示します。
ノード詳細には件数と割合を表示し、既存の安全判定用`label`は変更しません。
通常ブリッジ版は元姿勢の状態更新に追従し、単体版は保存時点の割合です。
割合は学習済み姿勢の構成比であり、成功確率ではありません。
旧形式の単体モデルは件数未収録のため従来色へ戻ります。割合表示にはモデル再生成が必要です。
今回のメッセージ拡張を反映する際は、関連ROSノードの再ビルド・再起動とViewerの再読み込みが必要です。
[変更点と検証](docs/releases/2026-09-16_l0_state_ratio_colors.md)を参照してください。

メッセージ定義を更新済みの環境で、静的な簡略版だけを追加する場合は次を使用します。

```bash
ros2 launch gng_vlut_system visualization_gng_static.launch.py \
  model_path:=/ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/vis_gng_static_L0.bin \
  topic_name:=/ToPoDualArm/Tmap_vis_static_L0 \
  frame_id:=ToPoDualArm/base_link
```

Viewerでは`/ToPoDualArm/Tmap_vis_static_L0`を選択します。単体版は保存時点の静的グラフで、
占有による安全状態の更新や軌道変換はありません。

`grasp_voxel_matcher_node`でこれらのトピックを入力として使う構成では、
発行をオンにするか、別のlaunchから発行ノードを起動してください。

- **GNGマップを送信**:
  ```bash
  ros2 launch gng_vlut_system topofuzzy_bridge.launch.py
  ```
- **アーム姿勢を送信**:
  ```bash
  ros2 launch gng_vlut_system topoarm_viewer_bridge.launch.py
  ```

### 環境点群からVLUTへの入力

`gng_viewer_bridge.launch.py`と併用し、YAMLで指定した点群をボクセル化します。

ROIは`ToPoDualArm/base_link`座標でボクセル化し、ViewerではTFを適用してworldへ表示します。
ロボットのyawに沿って格子が斜めになっても、占有範囲は環境点群と重なる構成です。
入力の`base_link`と`ToPoDualArm/base_link`は別frameとして扱い、末尾一致による自動読み替えは行いません。

現在のToPoDualArm設定は外部TF運用（`environment_voxelization.enable_static_tf: false`）です。
URDFが`base_footprint -> base_link`を配信するため、外部TFは次のルートへ与えます。
既存の`world -> ToPoDualArm/base_link`配信と固定設置用TFは終了してから切り替えてください。

```bash
python3 test_tf_publisher.py --world-frame world \
  --frame-id ToPoDualArm/base_footprint \
  --x 0.150 --y 0.0 --z 0.0 --yaw 1.5
```

入力frameのTFがない場合、`allow_unconnected_source_as_world: true`では入力座標をworldとみなします。
これは入力がworld基準であることを確認した場合だけの互換動作で、座標較正の代わりではありません。
通常は正しい入力frameとTFを与え、GNGとROIで同じ座標の意味を使用してください。
固定設置へ戻す場合は外部TFを停止し、同YAMLの固定TFを有効化します。

```bash
ros2 launch gng_vlut_system environment_to_vlut.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
```

ROI範囲は`/ToPoDualArm/Tmap_static`の全ノードのBBoxに、XYZ各面20 cmの余白を加えた範囲です。
余白はYAMLの`environment_voxelization.reachability_margin_x/y/z`で調整できます。
`reachability_map_topic: "Tmap_static"`はロボットnamespace内のマップを指定します。
マップ／必要なTFの取得まではROI配信を待機し、空文字列を指定した場合だけ従来の固定範囲を使用します。
ボクセル解像度は従来どおりVLUT基準です。[範囲計算の仕様](docs/TECHNICAL_SPEC.md#17-roi範囲の静的マップbbox追従)。

ToPoDualArmの自己領域除去が有効な構成では、環境ボクセルのトピックは次の2つです。

| トピック | 内容 |
| --- | --- |
| `/ToPoDualArm/roi_voxels` | 自己ロボット領域の除去前（旧`roi_voxel_ids_raw`） |
| `/ToPoDualArm/self_filter_roi_voxels` | 自己ロボット領域と除去余裕幅を除いた結果（旧`roi_voxel_ids`） |

自己マスクのURDF直方体（Box）は、初期構築時にセルとの交差・境界接触を判定してボクセル化します。
セル中心が形状の外側にある端部も収録対象です。判定用の軸・射影範囲はBoxごとに初期構築時だけ計算します。
関節角度変更後は従来どおり、初期セルをリンク姿勢で変換して更新します。
既存の更新処理には、格子と一致しない並進を伴う90°回転などで境界セルが不足する場合があります。
初期構築の変更を稼働中の自己マスクへ反映するには、再ビルド後に`self_recognition_viz_node`の再起動が必要です。

VLUT用の占有・危険ボクセルへの変換には除去後のトピックを使用します。
入力・出力名は`self_recognition.raw_environment_voxel_topic`と
`self_recognition.filtered_environment_voxel_topic`、後段の購読先は
`environment_voxelization.voxel_topic`で設定できます。
名称変更後は両方のlaunchを再起動し、Viewerで手動選択していた旧トピックを
新しい名前に変更してください。

### ROI登録とAiS-GNGの自己判定共有

実機MID-360のドライバ導入・IP設定・点群受信は[専用Docker環境の手順](../hardware/mid360/README.md)を参照。
GNGコンテナのビルドとは独立した導入。実機の出力トピックは公式既定の`/livox/lidar`。

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=topo_dual_arm_max_long.yaml enable_environment_voxelization:=true \
  enable_shared_roi_gng:=true enable_environment_world_index:=true \
  environment_input_topic:=/sim/lidar/points
```

- 入力: `environment_input_topic`。未指定時は機体YAMLの点群トピック。取得時刻のセンサ→world・ロボット基準TFが必須。
- 登録: 全点を一回だけworld_bucketへ登録 → ROIと自己領域の候補を細ボクセルへ一回登録 → 同じ占有配列の自己ラベル参照。bucket内に座標・元点番号を同居、ROIでのXYZ再読出し・別の自己用点群登録・点群再配信なし。
- 粗索引: `enable_environment_world_index:=true`。同じ索引をworld表示・ROI・共有storeへ渡す方式。幅・frame・表示トピックは機体YAMLの`environment_voxelization.world_index`。別world索引ノードとの同一入力への二重起動禁止。
- 単一ROI: 粗索引不要なら`enable_environment_world_index:=false`（既定）。直接ROI登録と自己ラベル共有だけ。粗索引の新規構築を追加しても総時間の短縮保証なし。
- VLUT: 自己領域を除いたROIセル。GNG: 同じ判定で自己点を除いた元点。ROI外の環境点もGNGの抽出対象、ROI外の自己点も除外対象。
- GNG入力登録: `voxel_idx`の共通キャッシュから`gng_set_registered_input`へ投入。元点→セル対応は1入力フレーム・格子条件ごとに1回、GNG内部の座標再量子化・再ソートなし。格子条件は解像度・原点・範囲・座標系・float32丸め規則、取得時TFはフレームごとの整合確認。ROI格子と異なる条件の強制共用なし。
- 学習互換: 入力上限・抽出方式・代表元点・重点学習の候補は従来どおり。選択集合が異なる場合も既登録セル番号の再利用、不正な登録結果は当該入力の学習保留。GNGノード近傍探索の索引は別用途のため維持。
- 自己領域: `self_recognition.self_exclusion_inflation`込み。単独GNGの追加膨張なしの方式とは除去境界が異なる点に注意。
- 起動: 同一containerのROI処理とCPU GNG。別の`self_voxel_filter_node`は起動なし。既存の`ais_gng.launch.py`との同一入力への二重起動禁止。
- GNG設定: `shared_gng_config:=at128.yaml`。CPUセンサ設定または絶対パス。出力はロボットnamespace内の`topological_map`等。曲面推定・データセットexporterの同時起動は対象外。
- 既定: 共有OFF・粗索引OFF。共有OFF時のGNG内部登録は従来どおり。共有は外部サンプラー公開付きCPU構成（`allow_external_sampler=ON`）のみ。単一利用ではキャッシュ管理・受渡しの追加コストあり、同一格子の複数利用で登録結果の再利用。総時間の短縮保証なし。

共有型・APIの変更後は、関連ノードを停止した状態で以下を実行。旧型のcomponentと新型の共有ライブラリの混在不可。

```bash
cd /ros2_ws
colcon build --packages-select voxel_idx pointcloud_sampling gng_cpu ais_gng gng_vlut_system fuzzy_voxel_grid \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Dallow_external_sampler=ON
source install/setup.bash
```

## 4. テストとデバッグ

自作テスト・ベンチコードはローカル保持・Git管理外、新規checkoutには未同梱。
未配置時の通常ビルドは継続。画面内QA・診断ノードは通常機能として維持。

### ボクセル色変化テスト

仮想的な占有・危険ボクセルをpublishして、RViz上での色の変化（安全:緑, 危険:黄, 衝突:赤）を確認します。

デフォルトでは、ワールド座標 `(x=20, y=20, z=30)` cm を中心に半径10cmの球が生成されます。

```bash
# デフォルト設定で実行
ros2 run gng_vlut_system voxel_status_test_publisher
```

中心座標や半径、オービット（周回）軌道などを変更したい場合は、パラメータで上書きできます。
```bash
# 例: 半径5cmの球を、Z=40cmの高さで、半径25cmの軌道上を20秒で周回させる
ros2 run gng_vlut_system voxel_status_test_publisher --ros-args \
  -p sphere_radius_cm:=5.0 \
  -p sphere_center_world_cm:="[0.0, 0.0, 40.0]" \
  -p sphere_orbit_radius_cm:=25.0 \
  -p sphere_orbit_period_s:=20.0
```

### 可視化スクリプト

学習結果をプロットして分析します。

- **関節角度の分布**:
  ```bash
  python3 scripts/visualize_joint_angles.py
  ```
- **可操作性の分布**:
  ```bash
  python3 scripts/visualize_manipulability.py --file build/manipulability_data.csv
  ```

## 5. 主要なノードとトピック (リファレンス)

### ノード
- `joint_state_mux_node`: RViz用と実機用の `JointState` を `/joint_states` に集約。
- `safety_monitor_node`: GNG/VLUT の安全状態を `/gng_viz` に表示。
- `self_recognition_viz_node`: 自己認識マスクを `/self_mask_viz` に表示。
- `topofuzzy_bridge_node`: `/topological_map` を Viewer 互換形式で publish。
- ...など

### トピック
- `/joint_states`: 現在のアーム姿勢。
- `/gng_viz`: GNGノードとエッジのマーカー。
- `/self_mask_viz`: 自己認識マスクのマーカー。
- `/self_recognition/voxel_mask`: 自己認識マスクの送信トピック。
- `/occupied_voxels`: self_recognition のボクセルIDを安全監視へ渡すトピック。
- `/danger_voxels`: occupied を外側に膨張したシェルを渡すトピック。
- ...など






最新版



ros2 launch gng_vlut_system gng_vlut_monitor.launch.py \robot_name:=topoarm \dir:=gng_results \id:=topoarm_full_v2 \mode:=STATIC \tag:=topoarm

など
