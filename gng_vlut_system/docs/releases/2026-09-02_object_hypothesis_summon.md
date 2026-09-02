# 2026-09-02 - 仮物体仮説による登録グラフ召喚

## 概要

認識器が完成する前でも、登録済み物体IDを仮選択して対応するGNGを召喚できるROS 2経路を追加した。
launch名は`object_hypothesis_summon.launch.py`とし、動作確認用途を示す語はlaunch名へ含めない。

## 変更

- `dataset_dir`以下から召喚可能な物体テンプレートを自動検出
- 登録IDのランダム選択と、topicによる手動ID選択
- 選択状態を`/object_hypothesis/summon_state`へ最新1件保持のJSONで配信
- 選択中IDのGNGを`/object_hypothesis/<template_id>/topological_map`へ配信
- 環境GNGの`/topological_map`と召喚結果のnamespaceを分離
- ID切替時に旧IDの空マップを配信し、表示済みグラフを消去
- `template_ids`による対象ID限定と`initial_template_id`による初期ID固定

## 挙動

ランダム仮説の`score`と`yaw_deg`は仮metadataであり、点群照合や位置姿勢推定の結果ではない。
このlaunchは登録済みGNGだけを召喚し、物体点群や位置変換済みグラフは生成しない。
`switch_interval_sec:=0.0`では自動切替を止め、初期IDまたは手動選択だけを反映する。

## 実行

```bash
ros2 launch gng_vlut_system object_hypothesis_summon.launch.py \
  dataset_dir:=/datasets \
  switch_interval_sec:=5.0
```

手動選択は次のtopicへ登録IDを送る。

```bash
ros2 topic pub --once /object_hypothesis/select std_msgs/msg/String \
  "{data: mug_complete}"
```

## 検証

- Releaseビルド
- 2種類のテスト用テンプレート検出とランダム切替
- topic指定による手動切替
- 選択中グラフの配信と旧グラフの空マップ配信
- テストで起動したROS 2プロセスと一時ファイルの残存なし、既存Dockerコンテナの稼働継続
