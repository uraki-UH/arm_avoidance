# 3Dレジストレーション方式の導入・検証手順

## 目的と適用範囲

遮蔽を含む単一視点の点群と事前テンプレートの照合について、候補方式を同一PCで導入・比較するための手順。

現行の最優先タスクは、平面対応とyaw候補で絞った非平面成分を加点評価する方式であり、並進を含む位置レジストレーションは導入しない。本書の方式は、将来の姿勢推定拡張または比較実験の候補。

TEASER++、SC2-PCR、TurboRegは対応点または特徴対応を入力とする。対応の生成時間と精度は以下の計測値に含まれない。G3Regは平面・クラスタ・線分の形状から候補対応を生成するため、現行の平面クラスタ・非平面成分との接続候補。

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

FilterRegはCUDA、PCL、OpenCV、glogへの依存があり、公開環境が旧Ubuntu・旧CUDAを前提とする。ホストのCUDA ToolkitとPCL版を確認し、Dockerまたは隔離環境で導入する。現時点では優先度を下げる。

FlashRegの公開リポジトリは取得できるが、検証時点では内容が空で実行コードを確認できなかった。clone後に`git log --oneline`と`git status`で実装の有無を確認する。

## G3Reg

G3Regは平面・クラスタ・線分をGaussian Ellipsoid Modelとして表し、Pyramid Compatibility Graphで複数姿勢仮説を検証するCPU方式。公式DockerfileはPCL 1.10、GTSAM 4.1.1、igraph 0.9.9をソースから構築するため、初回は時間とディスク容量を要する。

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

ヘッドレス実行器では、入力点数、前処理、グラフ構築、クリーク探索、姿勢解、検証、総時間を出力する。小物体への適用では、`min_range`、`max_range`、平面・クラスタ抽出の分解能、ノイズ境界をメートル単位で再調整する。

## 推奨する比較順

1. 現行方式の平面対応・yaw候補・非平面成分評価を基準として記録
2. 同じテンプレートと遮蔽フレームから局所特徴対応を作り、TEASER++を後段として評価
3. 対応数が多い条件でSC2-PCRとTurboRegを評価
4. 平面クラスタと非平面成分を楕円体特徴へ変換してG3RegのGEM/PAGORを評価
5. RGB-D、CAD、物体マスクを安定して用意できる場合だけFoundationPoseを別系統の比較対象として追加

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

方式ごとにライセンスと再配布条件を公式リポジトリで再確認する。ベンチマークの公称値を小物体・遮蔽環境の性能として扱わない。
