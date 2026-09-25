# CPU GNGの共通重点サンプリング

## 共通処理と条件別処理

把持・境界の評価器は`libgng_cpu.so`内部へ組込み、1入力単位で既存入力ボクセル上で評価。
ROS側は`fuzzrobo/libgng/api.h`の`gng_set_builtin_sampling`へ領域・境界位置だけを入力。
社内開発版では外部の評価器登録・旧添字APIも同じ重点抽選分布へ接続可能。
通常・unknown・重点の学習配分を共通化し、総学習回数とunknownの周期的な配分順を維持。
unknownの候補XYZ保持は従来処理を維持。単純な元点参照化は過去の全体時間比較で不利だったため、今回の対象外。
入力上限のrandom/uniform/stratified間引きは学習前の別工程であり、変更なし。

1. 既存voxel索引と、既存セル照合の最近傍結果を共有。再ボクセル化や追加の最近傍探索なし。
2. 各セルについて条件ごとの`cell_score`を評価。対象外は重み0。
3. `enable_point_weights`が立つ条件・セルだけ元点の`point_score`を評価。
4. 各条件の候補質量を正規化し、設定配分で重み付けして一つの累積分布へ集約。
5. 固定学習枠で抽選。通常枠の中は従来の全体／unknown配分を継続。

セル内で重みが一定なら、元点候補を展開せずセル範囲だけを保持。
条件ごとの全点重み配列・元点ソート・XYZコピーは不要。既存の入力索引構築自体は残る。
空の条件の配分は通常枠へ返却。条件同士が重なる点は、それぞれの配分分の確率を加算。
追加重点による再学習は共分散・支持・観測方向・統計用勝者イベントへ計上しない。

## 社内開発版と製品配布版

機能の有効化と、外部から評価器を差し替える権限は別のビルド設定。
`allow_external_sampler`はCMakeオプションであり、YAMLや実行時APIでの切替は不可。

| 構成 | `allow_external_sampler=ON`：開発用・既定値 | `allow_external_sampler=OFF`：製品配布用 |
| --- | --- | --- |
| 組込みの把持・境界、候補取得・診断 | 利用可能 | 利用可能 |
| 外部の規則・評価コールバック登録 | 利用可能 | 関数宣言と公開シンボルを除外 |
| 任意の元点番号・重みによる重点指定 | 利用可能 | 旧API2関数も除外 |
| `voxel_framework.hpp`・`builtin_sampling.hpp`・拡張用CMakeターゲット | 配布 | 配布対象外 |
| 内部の属性・ファジィ・履歴機能 | `enable_voxel_*`で選択 | 同じ設定で選択可能 |

除外する関数は`gng_set_sampling_rules`、`gng_set_priority_input`、`gng_set_weighted_priority_input`。
製品版は宣言を自作してもリンク不可。許可済み評価器の追加は社内ソースでの実装と再ビルドが必要。
任意の抽選器全体を動的ロードするプラグイン機構ではない。

組込みAPIは`gng_setPointCloud`の後、`gng_exec`の前に呼出し。
`gng_builtin_sampling_input`の把持AABB列、境界`Vec3`列・半径・各配分を入力し、配列は呼出し中に複製。
入力はGNGと同じ座標系・長さ単位。把持IDは1、境界IDは2で固定。
配分は有限・非負で合計1未満。各配列の件数上限は`node.num_max`、座標は有限、AABBは各軸でmin≦max。
境界点がある場合は半径とその二乗が有限かつ正。空の条件は通常枠へ返却。
入力置換・1回の実行後に失効し、`nullptr`で明示解除。不正入力・確保失敗は登録を解除して返値0。
再入・並行呼出しは非対応。ROS側の時刻・TF整合・期限判定は従来どおり。

製品版も入力点群・領域・配分などの承認済みデータは変更可能。
これは外部アルゴリズム注入を公開APIから禁止する構成であり、OS管理者によるバイナリ改造や置換への対策ではない。
製品には専用install成果物を配布し、開発用ヘッダーやソースを含むワークスペース全体を渡さない。

```bash
# ROS・必要な依存パッケージをsource済みのワークスペース直下
colcon --log-base log_product build \
  --build-base build_product --install-base install_product \
  --packages-select gng_cpu ais_gng \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -Dallow_external_sampler=OFF
```

既存の開発install先を流用せず、製品専用の新しいbuild/install先を使用。
開発用ヘッダーが残るprefixへの製品installは、書込み前に拒否し、自動削除はしない。
開発環境は既定ONを維持。設定変更時はCMakeキャッシュに注意し、両パッケージを再ビルド。
既存の外部規則／旧重点APIを使う利用側は製品版と非互換。ROSのYAML・トピックは変更なし。
[配布SDK・実GNGの検証](../../benchmarks/sampler_product_20260926/README.md)。

## 開発版の拡張点

| 評価器で参照可能な情報 | 内容・制限 |
| --- | --- |
| `num_points` | 入力上限・範囲フィルタ適用後の同一セル内元点数。センサー全点数ではない |
| `min_pos` / `max_pos` | GNG入力座標系のセルAABB。float量子化の保守的な余白付き |
| `node_id` / `node_frame` | 既存局所探索で得た最近傍ノードと生成世代。未対応は`UINT32_MAX` |
| `node_label` | 最近傍のGNGラベル。平面所属や意味分類結果そのものではない |
| `nearest_dist_sq` | 既存探索範囲内の最近傍距離二乗。ノード密度とは異なる指標 |
| `num_nodes` | 要求時だけ集計する、入力と同じセル内の有効GNGノード数 |
| `has_node_count` / `has_volume` | ノード数・体積の利用可否。voxel OFF時は体積密度として扱わない |
| `data` | 利用側で保持するラベル・平面サイズ等の属性表。評価器からの参照用 |

非平面・小平面などは、利用側のノード属性表を`node_id`で参照する評価器として追加可能。
再利用されたIDの誤対応を防ぐため`node_frame`も照合。前フレームの情報を使う場合は、利用側で時刻・座標系の失効判定が必要。
ラベル名を抽選器内の列挙値や分岐へ追加する必要はない。

追加の手順：

1. 条件の属性表と`cell_score`を用意。細かい位置判定が必要な場合だけ`point_score`も用意。
2. 一意の`id`、総学習回数に対する`ratio`、借用する`data`を規則へ設定。
3. `gng_setPointCloud`の後、`gng_exec`の前に`gng_set_sampling_rules`で規則を一括登録。
4. 抽出集合・選択確率・前処理と全体時間を検証。

組込み条件は`builtin_sampling.hpp`、抽選本体はCPU内部の`sampling.hpp`。
ROSの`AiSGNGComponent::prepare_priority_attention`は組込みAPIを使用。
開発用の規則生成例は`grasp_attention.hpp`の`sampling_rule`と、`boundary_attention.hpp`経由の同名関数。
2026-09-26の整理で旧`sampling_api.h`は`api.h`へ統合し、接続専用の`spatial_sampling.hpp`を廃止。
旧ヘッダーのincludeは統合先へ変更。関数・構造体のAPI名と抽選ロジックは変更なし。
現在の把持IDは1、境界IDは2。追加条件は衝突しないIDを使用。
新条件はC++評価器と必要な設定読込の追加が必要であり、任意のYAML項目だけで評価式を定義する機能ではない。

非平面・小規模平面・入力密度とノード密度の差による重点化は、今回の本番条件としては未追加。
拡張用の属性・点数・ノード数を使う試験は`sampling_test.cpp`にあり、実運用の評価式とは区別。
物体種別や動静分類・局所ノード密度の保証は、このサンプラーの役割ではない。

## 配分・重み・有効期間

`cell_score.weight`は元点当たりの重み。均一セルの質量は`weight × num_points`。
元点判定ありの場合は`cell_score.weight × point_score`を各元点の質量とする。
同じ重みの3点セルと30点セルは、元点均等なら質量比1:10。
セル均等を意図する条件は`num_points`で明示的に除算する。暗黙の密度補正は行わない。

各規則は有限・非負の重み、配分0は無効。配分合計は旧添字APIの枠も含め1未満。
最大64規則、規則IDの重複は拒否。ラベル数の上限ではなく、同時登録する評価器の上限。
規則の不正指定は登録を解除して返値0。評価中の例外・不正重みは、そのフレームの共通規則候補を除外し診断フラグを設定。
旧添字APIの有効候補と通常／unknown学習は継続。ROSでは警告を間引いて表示。
入力置換・1回の実行後に登録失効。`data`のメモリは`gng_exec`完了まで保持。
評価器からGNGの変更・API再入・並行呼び出しは非対応。
利用側が所有する属性・履歴の更新は可能。`data`に不変オブジェクトを渡した場合の変更は禁止。

## セル評価フレームワークのコンパイル時拡張

`fuzzrobo/libgng/voxel_framework.hpp`はROS非依存の共有ヘッダー。
GNG専用の抽選処理ではなく、入力セル→属性集計→任意の履歴更新→評価結果、の共通基盤。
属性型と結果型は利用側ポリシーの定義。GNGではサンプリング重み、FVGでは既存ラベル判定へ接続。
既存voxel／ManagedVoxelを入力として借用し、共通基盤による点群の再ボクセル化・別グリッドの構築はなし。
ただし任意の評価器内部の追加探索・計算を自動削減する機能ではない。

ビルド構成は`gng_cpu`のCMakeオプションで指定。いずれも既定OFF、ROSパラメータではない。

| オプション | 役割 |
| --- | --- |
| `enable_voxel_framework` | 属性集計と評価の経路 |
| `enable_voxel_fuzzy` | ポリシーのファジィ評価経路。属性集計が必要 |
| `enable_voxel_history` | 空間キーごとの属性履歴・更新経路。属性集計が必要 |

`build_features`はビルド構成そのもの。
`configured_features<enable_fuzzy, enable_history>`は利用側が必要機能だけを明示選択する型。
引数省略時はファジィ・履歴を要求しない。未ビルド機能を要求するとコンパイルエラー。
`features<...>`はビルド既定値とは独立した低層テンプレート構成で、単体試験などで使用可能。
全拡張OFFの`pipeline`は型の特殊化で`baseline`へ直結し、属性型・集計関数も実体化しない。
有効経路内は`if constexpr`で不要な更新・評価分岐と履歴保存型を除外。
空の型自体にはC++上のサイズがあるが、現在のローカル利用では最適化後の追加保存領域なし。
所有オブジェクトへ組み込む場合は空基底最適化やC++20の`[[no_unique_address]]`を利用。

ポリシーの契約：

| メソッド／型 | 必要な構成・内容 |
| --- | --- |
| `baseline(cell)` | 属性OFF時の既存結果 |
| `collect(cell)` | 属性ON時に1評価につき1回の属性取得 |
| `evaluate(attributes)` | ファジィOFF時の結果 |
| `fuzzy(attributes)` | ファジィON時の結果。具体的な式は利用側の定義 |
| `attributes_type`, `key_type`, `key_hash` | 履歴ON時だけ必要な所有属性型と安定キー |
| `key(cell)` | 履歴の空間キー。セル配列の添字`idx`はフレーム間の安定キーではない |
| `update(attributes, previous, elapsed_sec)` | 履歴ON時の更新。初観測はprevious=nullptr・経過秒0 |

履歴利用時は`begin_frame(epoch, stamp_sec, max_cells)`→各セルの`evaluate`→`end_frame`。
同一セルを同じフレームで二重評価する呼び方は拒否。共有属性から複数スコアを返す場合は評価結果の構造体化。
原点・セル幅・座標系変更時は利用側で`epoch`を更新し、移動センサーの座標整合も利用側で確保。
異なるepoch、同一／逆行時刻、容量設定変更で履歴失効。フレーム終了時に未観測キーを削除。
これは未観測領域の自由空間判定やノード削除ではない。旧オクルージョン判定の再導入なし。
`max_cells`は前回・今回の履歴が共存する間の保存件数上限。上限超過・評価例外・キー重複で履歴全体を失効し例外送出。
上限到達時の全セル走査・暗黙の拡張なし。平均O(1)のキー更新、終了時O(H)・メモリO(H)、Hは履歴保存件数。
GNGで別規則の例外が発生した場合も、利用側で`has_invalid_score`を確認してフレーム履歴を破棄する必要あり。

把持・境界とFVGの現行アダプターは`configured_features<>`を使用し、全機能ビルドでも履歴なし。
新しい密度差・小規模平面・非平面の評価式やメンバーシップ関数は本番に追加していない。
FVGの既存ラベル判定も維持し、未実装だったdegree計算を完成扱いにしない。

```bash
colcon build --packages-select gng_cpu ais_gng --cmake-args \
  -Denable_voxel_framework=ON -Denable_voxel_fuzzy=ON -Denable_voxel_history=ON
```

OFFへ戻す場合は3項目を明示的にOFFとして再ビルド。CMakeキャッシュは前回指定を保持。
開発版の利用側は`gng_cpu::voxel_framework`をリンクして構成を共有。`gng_cpu::gng_cpu`からもPUBLIC伝播。
製品版では同基盤を内部リンクだけに限定。外部SDKでテンプレート評価器の追加は不可。
同一プログラム内で異なるマクロ構成を混在させない。C++17以上が必要。
独立FVGパッケージのビルドには開発版ヘッダー・ターゲットが必要。製品SDK単独ではビルド不可。
FVGはヘッダー用のbuild依存のみ追加し、`libgng_cpu.so`への実行時依存なし。
既存の`fuzzy_voxel_grid/COLCON_IGNORE`は保持。通常のcolcon対象へ自動復帰しない。
通常の開発install先は3項目OFF・外部登録ON。
[共有基盤追加時点の実GNG・生成コード検証](../../benchmarks/voxel_framework_20260926/README.md)と、
[組込みAPI・配布制限追加後の検証](../../benchmarks/sampler_product_20260926/README.md)を区別。

## 計算コストと出力

占有セル数V、規則数K、実際の元点評価数R、候補エントリー数Eに対し、準備は概ねO(VK+R+E)。
セル属性だけの条件はRを増やさない。元点評価ループには、そのセルで詳細判定が必要な条件だけを渡す。
重点抽選は1回O(log E)。重なる条件が多い場合はEも増えるため、項目追加が無償になるわけではない。
無指定時は追加セル走査なし。ノード数は`enable_node_counts`要求時だけO(N log V)で集計し、同じ集計を全条件で共有。
この追加集計や利用側の属性表作成・近傍検索のコストも、機能評価の対象。

`gng_get_sampling_stats`でセル／元点評価数・候補数・重点学習回数・不正評価の有無を取得可能。
`gng_get_sampling_points`は指定規則の候補を購読時用に展開。実際に学習した点の時系列ではない。
ROSの`/downsampling/grasp`は共通候補を再利用し、現在はGNG入力範囲フィルタ適用後の点を配信。
条件ごとの自動トピック生成なし。削除した`/downsampling/nonplane`の復活なし。

2026-09-25の共通化では候補条件・配分を維持しつつ抽選順・乱数消費順を変更。
2026-09-26の組込みAPIへの移行では、変更前と同じ乱数条件で候補順・グラフ出力の一致を検証。
既存ROS設定は維持。共通APIはCPU専用であり、`gng_cpu`・`ais_gng`の再ビルドと使用中GNGの再起動が必要。
検証条件・計測結果：[共通化の検証](../../benchmarks/common_sampling_20260925/README.md)。
