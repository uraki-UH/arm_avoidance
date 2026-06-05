# GNG-Classification

## 必要なライブラリをインストール
`pip3 install -r requirements.txt`

## セットアップ
`cd ~/ros2_ws/src/AiS-GNG/gng_classification && mkdir datasets`

でデータセットフォルダを作成

## 実行

- サンプリング
  `ros2 run gng_classification sampling_dataset`
  
  EnterでrosbagのStart/Stopを切り替えられる。
  
  Stopした際に、HumanIDを入力する様に求められるので、","で区切って入力する。

  Datasetsは，ros2_ws/install/gng_classification/share/gng_classification/datasetsに保存される。

- 学習
  `cd ~/ros2_ws/src/AiS-GNG/gng_classification/`
  `python3 scripts/nn_learning.py`