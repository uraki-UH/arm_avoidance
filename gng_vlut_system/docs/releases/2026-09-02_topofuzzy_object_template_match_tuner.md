# 2026-09-02 - TopoFuzzy物体テンプレート照合チューナー

## 概要

TopoFuzzy Viewerの`Analyze`右隣へ`Match`タブを追加し、物体GNGテンプレート照合の
matcher・validatorパラメータを実行中に調整できるようにした。
画面本体と型・parameter定義は新規の`features/templateMatching`配下へ分離した。

## 変更内容

- 姿勢探索、node、平面、graph・scale、反証、確定判定の設定をグループ別に表示する。
- yawは常に全周探索とし、GUIには分解能の`Yaw full-turn step`だけを表示する。
- roll/pitchはそれぞれ対称な`±tolerance`と共通刻みをGUIから設定する。
- `Match`タブ選択時に全画面モーダルを開き、広い画面では設定グループを3列で表示する。
- モーダルを閉じた後はタブ内の`Open`から再表示でき、Esc、背景クリック、右上ボタンで閉じられる。
- `Load`で実行中ノードの値を取得し、`Apply`で変更を反映する。
- `Reset`でコード内既定値を編集状態へ戻す。ROSへの反映は`Apply`時だけ行う。
- `Open`と`Save`で調整値をローカルJSON profileとして入出力する。
- matcher・validatorの対象ノード名を画面上で指定可能にした。
- `viewer_template_match_node`を追加し、許可したparameterだけを専用RPCで中継する。
- matcherとvalidatorへ動的parameter callbackを追加し、相互依存する範囲を原子的に検証する。
- validatorが更新を拒否した場合、先に反映したmatcher値を適用前の値へ戻す。
- `viewer_stack.launch.py`から専用RPCノードを起動する。
- タブ数に応じてタブ列の幅を自動配分し、5タブを1行に維持する。

## 挙動影響

- 照合ノードを再起動せずにファジー評価範囲、重み、scale、反証、確定条件を調整できる。
- 不正な範囲や型、未許可parameter、無効なノード名は反映せずエラーを返す。
- `state_publish_hz`はタイマー周期を安全に作り直さないため、実行中変更の対象外とする。
- JSON profileはブラウザのローカル入出力であり、`/datasets`へのサーバ側YAML保存ではない。
- topic、message定義の変更はない。内部RPC topicは既存の
  `/viewer/internal/rpc/request`と`/viewer/internal/rpc/response`を使用する。

## 検証

- frontend: `npm run lint`成功。error・warningなし。
- frontend: `npm run build`成功。2343 modulesをproduction buildした。
- headless Chromeで1600x861と390x844を確認し、dialogがviewport内に収まり、操作部品の重なりがないことを確認した。
- backend: `colcon build --packages-select gng_vlut_system topo_fuzzy_viewer --cmake-args -DCMAKE_BUILD_TYPE=Release`成功。
- 隔離`ROS_DOMAIN_ID=211`でmatcher、validator、専用RPCノードを起動し、設定取得RPCの応答を確認した。
- `normal_weight`の正常な動的更新と元値への復元を確認した。
- `max_normal_angle_full_deg=40.0`、`max_normal_angle_partial_deg=35.0`の矛盾した更新が拒否されることを確認した。
- matcher更新後にvalidatorへ不正な閾値を送信し、`VALIDATOR_REJECTED`応答とmatcher値のロールバックを確認した。

## 実行条件

既存の`viewer_stack.launch.py`を再起動すると専用RPCノードが含まれる。
matcherとvalidatorは`object_template_matching.launch.py`などで別途起動しておく。
