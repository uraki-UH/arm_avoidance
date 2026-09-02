# 3Dレジストレーション方式の導入・検証手順

## 目的と適用範囲

遮蔽を含む単一視点の点群と事前テンプレートの照合について、候補方式を同一PCで導入・比較するための手順。

現行の最優先タスクは、平面対応とyaw候補で絞った非平面成分を加点評価する方式であり、並進を含む位置レジストレーションは導入しない。本書の方式は、将来の姿勢推定拡張または比較実験の候補。

TEASER++、SC2-PCR、TurboRegは対応点または特徴対応を入力とする。対応の生成時間と精度は以下の計測値に含まれない。G3Regは平面・クラスタ・線分の形状から候補対応を生成するため、現行の平面クラスタ・非平面成分との接続候補。

追加調査候補として、低重なり対応の`GeoTransformer`と`PREDATOR`、回転不変特徴の`RoITr`、対応点の外れ値除去に特化した`PointDSC`、CPUで動く古典的な`Super4PCS`と`Open3D FGR`、密対応の信頼度推定を行う`DGR`を加える。比較基準として`FPFH + RANSAC`と`ICP`も含める。これらは今回の時点では未取得・未実行。

## 検証環境

| 項目 | 検証環境 |
| --- | --- |
| GPU | NVIDIA GeForce RTX 4060 Laptop GPU、8 GB |
| NVIDIA driver | 595.84 |
| CUDA driver API | 13.2 |
| Python | 3.12 |
| 取得先 | `/tmp/registration_eval_YYYYMMDD` |

ホストのGPUが使えるかは、次で確認する。

```bash
nvidia-smi
```

このPCのDockerはCDI設定が未完了であり、`docker run --gpus all`ではGPUを公開できない。PyTorchのGPU実行はホスト側Pythonから可能。Docker内GPUが必要な方式は、別PCではNVIDIA Container ToolkitまたはCDI設定を先に確認する。

## 共通の取得手順

ワークスペースを汚さないため、取得先は一時領域に限定する。

```bash
eval_root=/tmp/registration_eval_$(date +%Y%m%d)
./gng_vlut_system/tools/registration_eval/clone_registration_repositories.sh "$eval_root"
```

このスクリプトはTEASER++、G3Reg、SC2-PCR、TurboReg、FoundationPose、FilterReg、FlashRegを取得する。TEASER++だけはsubmoduleを含めて取得する。

## 共通の自動化スクリプト

| スクリプト | 用途 |
| --- | --- |
| `clone_registration_repositories.sh` | 全候補の一時領域への取得 |
| `build_teaser.sh` | TEASER++のビルドとCTest |
| `setup_sc2_turboreg.sh` | CUDA対応PyTorch、SC2-PCR、TurboRegの導入確認 |
| `g3reg_headless.patch` | G3RegへのGUIなし計測器追加 |
| `prepare_foundation_pose_input.py` | `rgbd_png`保存済みGNGデータセットのFoundationPose単一フレーム入力化 |

## 追加候補

| 方式 | 主な役割 | 現行構成との相性 | 優先度 |
| --- | --- | --- | --- |
| GeoTransformer | 低重なり点群からsuperpoint対応を生成し、RANSACなしで姿勢推定 | 平面・非平面候補の前段として有望。ただしGPUと学習済み重みが必要 | 高 |
| PREDATOR | 重なり領域を推定して低重なり対応を生成 | 遮蔽された物体の観測部分を探す前段として有望。GPUと学習済み重みが必要 | 高 |
| RoITr | PPFベースの回転不変特徴と低重なり対応 | yaw総当たりを減らせる可能性がある。GPU、CUDA拡張、重みが必要 | 中〜高 |
| PointDSC | 対応点の空間整合性から外れ値を除去 | 既存matcherの対応点を後段へ渡せる。学習済みモデルとGPUが必要 | 中〜高 |
| Super4PCS | 対応点なしのCPU大域位置合わせ | 学習不要の比較基準。点群の部分重なりに対応するが、物体候補単位では重い可能性 | 中 |
| Open3D FGR | FPFHまたは既存対応点から高速大域位置合わせ | 最小の古典ベースラインとして導入しやすい。特徴生成の品質に依存 | 中 |
| FPFH + RANSAC | FPFH対応から外れ値を含む大域姿勢を推定 | 学習不要の標準比較基準。試行回数により計算時間が増える | 高 |
| ICP point-to-plane | 初期姿勢から最近傍対応を反復して局所精密化 | 大域探索後の共通後段。初期姿勢なしでは物体候補の選択に使えない | 高 |
| Generalized ICP | 点の共分散を使う局所精密化 | 平面クラスタの分散情報を活かせる可能性があるが、やはり初期姿勢が必要 | 中 |
| DGR | 対応の信頼度推定、重み付きProcrustes、姿勢精密化 | 点群対応を大量に作れる場合に有効。MinkowskiEngine等のGPU依存が重い | 低〜中 |

調査根拠は、GeoTransformerが低重なり向けの幾何特徴とRANSAC不要の姿勢推定を提案していること、PREDATORが重なり領域を明示的に推定すること、RoITrがPPFベースの回転不変表現を使うこと、PointDSCが空間整合性を使って外れ値を除去すること、Super4PCSが部分重なりを扱うCPU方式であることによる。Open3D FGRは対応点入力にも使える実装があり、DGRは対応信頼度と重み付きProcrustesを一体化している。
調査根拠は、GeoTransformerが低重なり向けの幾何特徴とRANSAC不要の姿勢推定を提案していること、PREDATORが重なり領域を明示的に推定すること、RoITrがPPFベースの回転不変表現を使うこと、PointDSCが空間整合性を使って外れ値を除去すること、Super4PCSが部分重なりを扱うCPU方式であることによる。Open3D FGRは対応点入力にも使える実装があり、DGRは対応信頼度と重み付きProcrustesを一体化している。Open3Dの標準構成でもFPFH対応を使ったRANSAC/FGRを大域初期化に、point-to-plane ICPを局所精密化に使う。

## 追加候補の優先順

1. `FPFH + RANSAC`を学習なしの大域基準として追加
2. `Open3D FGR`を対応点入力の簡易CPU基準として追加
3. `Super4PCS`を対応点生成なしのCPU大域基準として追加
4. 各大域方式の出力を`ICP point-to-plane`で精密化して比較
5. `PointDSC`を既存matcherの対応点後段として評価
6. GPU環境を整えた後、`GeoTransformer`または`PREDATOR`を低重なり対応生成器として評価
7. yaw総当たり削減が必要になった場合に`RoITr`を評価
8. 対応数が多い条件だけ`DGR`を評価

スクリプトはワークスペースのルートから実行する。GPUを使うSC2-PCRとTurboRegは、取得後に次で導入確認する。

```bash
./gng_vlut_system/tools/registration_eval/setup_sc2_turboreg.sh \
  "$eval_root/py_sc2" "$eval_root/SC2-PCR" "$eval_root/TurboReg"
```

## TEASER++

対応点に外れ値が多い場合のロバスト姿勢解。CMakeでビルドする。

```bash
./gng_vlut_system/tools/registration_eval/build_teaser.sh \
  "$eval_root/TEASER-plusplus"
```

小物体では、対応点ノイズ上限を物体スケールに合わせてメートル単位で設定する。テンプレートと観測点群の特徴対応は別に生成する必要がある。

### 2026-09-02時点の実測

既存のmug点群にZ軸0.5 rad回転と並進`(0.10, 0.02, 0.03)`を適用し、対応点の一部へ1 m級の外れ値を加えた。TEASER++は外れ値を除外し、全条件で既知の変換を復元した。

| 対応点数 | 外れ値 | 総時間 |
| ---: | ---: | ---: |
| 100 | 0% | 23.6 ms |
| 100 | 30% | 15.9 ms |
| 500 | 50% | 21.5 ms |
| 1,000 | 50% | 48.5 ms |

現行の平面クラスタ・非平面成分から対応候補を作り、誤対応を含んだまま姿勢解だけをTEASER++へ渡す構成に適する。TEASER++自体は対応生成を行わないため、遮蔽された実フレームでの対応再現率は別途評価が必要。

## SC2-PCR

SC2-PCRはGPU上で対応点の二次互換性から外れ値を除外して姿勢を解く。公式環境は古いPyTorch版を固定しているが、CUDA対応PyTorchで動作確認できる。

```bash
python3 -m pip install --target "$eval_root/py_sc2" --upgrade torch \
  --index-url https://download.pytorch.org/whl/cu128
PYTHONPATH="$eval_root/py_sc2" python3 -c \
  'import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))'
```

公式実装の`post_refinement`は、`inlier_threshold == 0.10`以外を1.2 mとして扱う固定値を持つ。数十cmの物体でそのまま使うと不適切なので、利用前にスケール依存の閾値へ修正し、単体テストを追加する。

## TurboReg

TurboRegはGPUで多数の対応点の最大クリークを探索する後段方式。対応点生成には、同梱例と同様にFPFHなどを使用する。

```bash
PYTHONPATH="$eval_root/py_sc2" python3 -m pip install \
  --target "$eval_root/py_sc2" --no-deps --no-build-isolation \
  "$eval_root/TurboReg/bindings"
```

`TurboRegGPU`はCUDAデバイスを要求する。小物体ではFPFHの法線半径・記述子半径・対応許容距離を物体寸法と点密度に合わせる。大規模対応では有望だが、数十件程度の非平面成分に対してはGPU起動や特徴抽出の固定費が支配的になり得る。

## FoundationPose・FilterReg・FlashReg

FoundationPoseはRGB、深度、カメラ内部パラメータ、CAD、対象物マスク、学習済み重みを必要とする。環境構築前にGPUとCUDA Toolkit、重みとデモデータの取得可否を確認する。物体マスクを安定して供給できる場合にだけ、幾何方式とは別系統の比較対象として導入する。

`ObjectGngDatasetExporter`は、organized RGB-D点群と対応する`CameraInfo`を受信して保存した場合、`rgbd_png`形式のdepth PNG、color PNG、内部パラメータをGNGデータセットへ記録する。次の準備器は、その保存物と外部マスクをFoundationPoseの`run_demo.py`が読む単一フレーム構成へ変換する。

```bash
python3 gng_vlut_system/tools/registration_eval/prepare_foundation_pose_input.py \
  /datasets/basket_gng_template.json.gz \
  --mesh-file /datasets/basket.obj \
  --mask-file /datasets/basket_mask.png \
  --output-dir /tmp/foundation_pose_basket \
  --foundation-pose-dir "$eval_root/FoundationPose"
```

通常はセグメンテーション器またはユーザー指定の`--mask-file`を使う。`--allow-depth-mask`は、背景を含まない単体撮影で変換経路だけを確認する暫定用途であり、実環境の物体マスクとして使わない。準備後の実行例はスクリプトが表示する。`FoundationPose`の推定結果は6D姿勢仮説として扱い、現行GNGの平面・非平面・共分散評価で検証してからテンプレートを召喚する。

`ToPo-FUZZY_Manipulation_v1.html`の深度カメラ設定には、`FoundationPose RGB-D ZIP保存`を用意する。遮蔽込みの16-bit depth、固定グレーのRGB、object系ラベルのmask、`cam_K.txt`、入力manifestをZIPへ出力する。ZIPを展開して`run_demo.py`の`--test_scene_dir`へ渡し、対応CAD meshを`--mesh_file`へ指定する。HTMLのRGBは実写ではないため、この経路はFoundationPoseの入出力接続と遮蔽条件の検証に限定する。

RGB検証モードは`固定グレー`、`白色ランダムノイズ`、`近似ピンクノイズ`から選ぶ。ノイズは全画素でグレースケール値を共有し、物体ラベル・深度・maskとは独立である。固定したseedごとに同一姿勢の推定結果を記録し、seed間の姿勢誤差と成功率を比較して色への依存度を評価する。これは色に対するストレス試験であり、FoundationPoseの深度専用方式を意味しない。

`ToPo-FUZZY_Manipulation_v1.html`で`合成深度点群`を選び点群を生成すると、ブラウザ直結の`rosbridge_server`へ現在の投影を自動送信する。配信先は`/topo_fuzzy/rgbd`配下の`/color`、`/depth`、`/mask`、`/camera_info`であり、前3者は`sensor_msgs/msg/Image`、`camera_info`は`sensor_msgs/msg/CameraInfo`である。`color`は`rgb8`、`depth`はミリメートル単位の`16UC1`、`mask`は`mono8`である。4トピックのheader stampとframe_idは一致し、frame_idは`foundation_pose_camera`となる。深度カメラ設定の`RGB-D camera topicを自動出力`をOFFにすると、`semantic_points`などのPointCloud2だけを出力する。表面点群または取り込みフレームへ切り替えた場合もRGB-D topicを停止する。

FilterRegはCUDA、PCL、OpenCV、glogへの依存があり、公開環境が旧Ubuntu・旧CUDAを前提とする。ホストのCUDA ToolkitとPCL版を確認し、Dockerまたは隔離環境で導入する。現時点では優先度を下げる。

FlashRegの公開リポジトリは取得できるが、検証時点では内容が空で実行コードを確認できなかった。clone後に`git log --oneline`と`git status`で実装の有無を確認する。

## G3Reg

G3Regは平面・クラスタ・線分をGaussian Ellipsoid Modelとして表し、Pyramid Compatibility Graphで複数姿勢仮説を検証するCPU方式。公式DockerfileはPCL 1.10、GTSAM 4.1.1、igraph 0.9.9をソースから構築するため、初回は時間とディスク容量を要する。

### 2026-09-02時点の実測

同梱Livoxデータをヘッドレス実行器で処理したところ、総時間は約79 msだった。内訳はグラフ構築31.6 ms、クリーク探索10.8 ms、姿勢解23.6 ms、検証4.3 msである。

現在の物体テンプレート`mug_source.pcd`（4,280点）を生点群のままG3RegのLiDAR前処理へ入力すると、範囲・クラスタ・FPFHのパラメータを物体サイズ向けに調整しても前処理中に異常終了した。このため、現時点ではG3Regの生点群フロントエンドを採用しない。

一方、既存のGNG候補から対応点を作ってG3Regの後段だけを呼ぶ評価器では、既知の回転・並進を正しく復元した。mugの対応点数別のPAGOR総時間は次の通り。

| 対応点数 | 総時間 |
| ---: | ---: |
| 10 | 11.0 ms |
| 50 | 10.8 ms |
| 100 | 14.5 ms |
| 500 | 34.6 ms |
| 1,000 | 171.4 ms |

したがって、現行の平面クラスタ・非平面成分から数十〜数百件の対応を作れるなら、G3RegはCPU後段の比較対象として有望。対応生成、遮蔽、誤対応を含む実データ評価は未完了であり、まだ本番の物体召喚経路には接続していない。

```bash
docker build -t registration_eval_g3reg:local "$eval_root/G3Reg"
support_root="$(pwd)/gng_vlut_system"
docker run --rm -it \
  -v "$eval_root/G3Reg:/G3Reg" \
  -v "$support_root:/evaluation_support:ro" -w /G3Reg \
  registration_eval_g3reg:local bash
```

このコマンドはワークスペースのルートで実行する。`support_root`は本リポジトリの`gng_vlut_system`への絶対パス。

コンテナ内でビルドする。

```bash
git -C /G3Reg apply \
  /evaluation_support/tools/registration_eval/g3reg_headless.patch
cp /evaluation_support/tools/registration_eval/g3reg_correspondence.cpp \
  /G3Reg/examples/headless_corresp.cpp
git -C /G3Reg apply \
  /evaluation_support/tools/registration_eval/g3reg_correspondence.patch
cmake -S /G3Reg -B /G3Reg/build
cmake --build /G3Reg/build -j"$(nproc)"
```

公式`demo_reg`はPCLの可視化ウィンドウを開く。GUIがないPCでは、本リポジトリの`g3reg_headless.patch`で表示処理を省いた実行器を追加し、同梱データを実行する。

```bash
/G3Reg/bin/headless_reg \
  configs/hit_ms/gem_pagor.yaml \
  examples/data/livox/source.pcd \
  examples/data/livox/target.pcd
```

対応点ベースの後段だけを測る場合は、同じ順番の点を対応点として使う評価用ヘルパーを実行する。これは現在のmatcherを置き換えるものではなく、G3Regの姿勢解部分の単体評価用である。

```bash
/G3Reg/bin/headless_corresp \
  /datasets/mug_source.pcd /datasets/mug_target.pcd 100 pagor
```

ヘッドレス実行器では、入力点数、前処理、グラフ構築、クリーク探索、姿勢解、検証、総時間を出力する。小物体への適用では、`min_range`、`max_range`、平面・クラスタ抽出の分解能、ノイズ境界をメートル単位で再調整する。

## 推奨する比較順

1. 現行方式の平面対応・yaw候補・非平面成分評価を基準として記録
2. FPFH + RANSAC、Open3D FGR、Super4PCSを学習なしの大域基準として評価
3. 各大域推定結果をpoint-to-plane ICPで精密化し、Generalized ICPも比較
4. 同じ対応点をTEASER++、PointDSC、TurboReg、SC2-PCRへ渡して後段性能を比較
5. 平面クラスタと非平面成分を楕円体特徴へ変換してG3RegのGEM/PAGORを評価
6. GPU環境を整えた後、GeoTransformer、PREDATOR、RoITr、DGRを評価
7. RGB-D、CAD、物体マスクを安定して用意できる場合だけFoundationPoseを別系統の比較対象として追加

評価は、姿勢誤差だけでなく、特徴抽出を含む処理時間、正しい物体候補を残す再現率、背景平面のみで誤検出しない適合率、遮蔽率ごとの失敗率を記録する。

## ライセンスと導入上の注意

| 方式 | 用途 | 導入判断 |
| --- | --- | --- |
| TEASER++ | 対応点後段のロバスト姿勢解 | 直近の比較候補 |
| G3Reg | 形状セグメントからの大域照合 | 非平面成分との統合候補 |
| SC2-PCR | 少数〜中規模対応のGPU後段 | 閾値修正後に限定評価 |
| TurboReg | 多数対応のGPU最大クリーク | FPFH等の対応生成と一体で評価 |
| FoundationPose | RGB-D・CAD・物体マスクによる6D姿勢 | GPU環境とモデル重みの整備後 |
| FilterReg | 確率的レジストレーション | 旧CUDA/PCL依存のため優先度低 |
| FPFH + RANSAC | 学習なしの大域姿勢推定 | 標準比較基準。対応数と試行回数を固定して評価 |
| Open3D FGR | 高速大域姿勢推定 | CPU基準。対応点またはFPFHを入力 |
| Super4PCS | 対応点なしの大域姿勢推定 | CPU基準。部分重なりの評価に使用 |
| ICP / Generalized ICP | 局所姿勢精密化 | 大域推定結果を初期値として比較 |

方式ごとにライセンスと再配布条件を公式リポジトリで再確認する。ベンチマークの公称値を小物体・遮蔽環境の性能として扱わない。
