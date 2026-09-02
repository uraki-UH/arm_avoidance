# 2026-09-02 - TopoFuzzy物体テンプレート照合チューナー

## 概要

TopoFuzzy Viewerの`Analyze`右隣へ`Match`タブを追加し、物体GNGテンプレート照合の
matcher・validatorパラメータを実行中に調整できるようにした。
画面本体と型・parameter定義は新規の`features/templateMatching`配下へ分離した。

## 変更内容

- 評価設定を`Roll tolerance`、`Pitch tolerance`、`Shape tolerance`、`Minimum visible ratio`、
  `Scale tolerance`、`Contradiction limit`、`Recognition threshold`、`Confirmation time`の8項目へ整理する。
- 8項目を`Orientation`、`Evidence`、`Rejection`、`Decision`の4グループで表示する。
- yaw全周探索の分解能は評価項目から分離し、YAML側の実装設定とする。
- roll/pitchはtoleranceが0度なら軸を固定し、0度より大きければ許容範囲を探索する。
- `Match`タブ選択時に全画面モーダルを開き、広い画面では設定グループを3列で表示する。
- `Match`タブのクリックで直接モーダルを開き、同じタブの再クリックでも再表示できる。
- Esc、背景クリック、右上ボタンでモーダルを閉じられる。
- Matchモーダル内の本文、入力値、操作ボタン、見出しを従来の約2倍へ拡大する。
- `Load`で実行中ノードの値を取得し、`Apply`で変更を反映する。
- モーダル表示時には自動取得せず、`Load`を押した場合だけ取得する。応答がない場合は3秒で停止してエラーを表示する。
- `Reset`でコード内既定値を編集状態へ戻す。ROSへの反映は`Apply`時だけ行う。
- `Open`と`Save`で調整値をローカルJSON profileとして入出力する。
- matcher・validatorの対象ノード名を画面上で指定可能にした。
- `viewer_template_match_node`を追加し、8つの高水準parameterだけを専用RPCで中継する。
- matcherとvalidatorへ動的parameter callbackを追加し、相互依存する範囲を原子的に検証する。
- validatorが更新を拒否した場合、先に反映したmatcher値を適用前の値へ戻す。
- `viewer_stack.launch.py`から専用RPCノードを起動する。
- タブ数に応じてタブ列の幅を自動配分し、5タブを1行に維持する。

## 挙動影響

- 照合ノードを再起動せずに形状許容、可視率、scale、反証、認識しきい値、確定時間を調整できる。
- node次数をshape scoreから外し、法線と`rho`だけを固定比率で合成する。
- edge一致率と平面支持度は大きい方を配置根拠とし、二重加点しない。
- 候補scoreを`cbrt(shape_score * relation_score * visible_ratio)`へ一本化する。
- scaleと反証はmatcherの棄却条件へ集約し、validator側の重複条件を廃止する。
- 不正な範囲や型、未許可parameter、無効なノード名は反映せずエラーを返す。
- `state_publish_hz`はタイマー周期を安全に作り直さないため、実行中変更の対象外とする。
- JSON profileはブラウザのローカル入出力であり、`/datasets`へのサーバ側YAML保存ではない。
- topic、message定義の変更はない。内部RPC topicは既存の
  `/viewer/internal/rpc/request`と`/viewer/internal/rpc/response`を使用する。

## 検証

- frontend: `npm run lint`成功。error・warningなし。
- frontend: `npm run build`成功。2343 modulesをproduction buildした。
- headless ChromeでMatch dialogに4グループ、number入力8個、slider 8個だけが表示されることを確認した。
- backend: `gng_vlut_system`のReleaseビルド成功。
- backend: `viewer_template_match_node`の個別ビルド成功。
- `topo_fuzzy_viewer`全体ビルドは、今回の変更外である`viewer_ws_gateway_node.cpp`の
  `detectType`引数不整合により失敗する状態を確認した。
- 隔離`ROS_DOMAIN_ID=212`で8 parameterの取得、正常更新、範囲外値の拒否を確認した。
- 隔離`ROS_DOMAIN_ID=213`で専用RPCを起動し、設定取得応答が新8 parameterだけを含むことを確認した。

## 実行条件

既存の`viewer_stack.launch.py`を再起動すると専用RPCノードが含まれる。
matcherとvalidatorは`object_template_matching.launch.py`などで別途起動しておく。
