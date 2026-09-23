# 2026-09-23 - GNG launchの入力トピック上書き修正

## Summary

`input_topic:=/lidar_points`指定時に、GNGがYAML既定の`/scan`を待ち続ける問題の修正。

## Changed

- センサーYAMLから読込済みの`ais_gng_node.ros__parameters`と短名変換結果を辞書として受け渡し。
- 共通設定・センサー設定・上書き辞書のセレクターを揃え、明示launch引数を末尾へ配置。
- 回帰テストをPythonでの辞書合成から、隔離ドメイン229でのROS自身のパラメータ解決へ変更。

## Added

- CPU・GPU設定での入力トピック上書きと保存ノードへの一致確認。
- 入力引数省略時に、YAMLの複数入力トピックを維持するテスト。

## Fixed

- 先頭の共通設定`/**`と後続のセンサー設定`ais_gng_node`が混在し、明示した入力先がセンサー既定値へ戻る不具合。
- 同じ理由で失敗する`use_node_rho_for_seed_order`の明示指定と、旧名より短名設定を優先する処理。

## Removed

機能の削除なし。

## Behavior Impact

共通設定よりセンサー設定、対応する明示launch引数を優先。
入力引数を省略した場合は、従来どおりYAMLの`input.topic_names`を使用。
YAML値・GNG計算処理・C++・メッセージ定義の変更なし。

## Topics / Params / Messages

- `input_topic`: GNGの`input.topic_names`を単一トピック配列で上書き。
- `source_point_cloud_topic:=auto`: 同じ明示入力、またはYAMLの先頭入力を保存用ノードへ転送。
- 新しいトピック・パラメータ・メッセージの追加なし。

## Verification

- 修正前の実ROSテストで、入力トピックCPU／GPU、rho指定、短名優先2条件の計5失敗を再現。
- 修正後はテスト11件が成功。テスト初回のHumble API差異は`get_parameters_by_prefix`へ修正後に再実施。
- 指定のlaunchを35秒限定で実行。`input.topic_names ["/lidar_points"]`、初期化成功、約16万入力点・約1.8万ノードの継続計算を確認。
- 別ターミナル側のGNG起動を検出したため、そのプロセスは維持。検証用GNG終了後、入力購読・マップpublisherが各1つのGNGであることを確認。
- 読み取り確認で`/topological_map`の5メッセージを受信。frame番号1842→1858、各18,602〜18,636ノード。最初の購読プローブのExecutor指定不足は、専用Contextに対応するExecutorへ修正して再確認。
- 起動した検証用launch 2回と全子ノード・プローブ・テストは終了済み。既存Viewer・bag再生・別ターミナル側GNGは停止／再起動なし。
- `git diff --check`成功。現在のコンテナはlaunchがソースへのsymlinkのため再ビルド不要。

実施コマンド：

```bash
docker compose exec -T gng_cpu bash -lc 'source /ros2_ws/install/setup.bash && PYTHONDONTWRITEBYTECODE=1 timeout -s INT -k 5 45 python3 /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/test_clustering_yaml_launch.py -v'
```

コンテナ内の起動検証では、`mktemp -d /tmp/gng-input-topic-XXXXXX`で作成した`/tmp/gng-input-topic-1lFFeP`を`TMPDIR`、その`logs`を`ROS_LOG_DIR`として指定。

```bash
timeout -s INT -k 8 35 ros2 launch ais_gng ais_gng.launch.py \
  backend:=cpu lidar:=at128.yaml input_topic:=/lidar_points
```

上限到達時のSIGINTで子ノードも正常終了。追加の同コマンド起動はPythonの`subprocess.Popen(..., start_new_session=True)`で所有し、`finally`から所有プロセスグループだけをSIGINTで停止。
最終の読み取りプローブは`timeout -s INT -k 3 12 python3 -`で実施し、購読・Executor・Contextを終了。
一時ディレクトリ内の検証ログ・launch生成パラメータは削除済み。既存ログへの削除なし。

## Risk / Notes

- GPUはパラメータ解決のみ検証。実点群での実行はCPUのみ。
- 現在のbagで`map`から`hesai_lidar`へのTF未登録警告あり。GNG入力の停止原因とは別件で、座標変換の修正は今回の対象外。
- 共通設定は`/**`、センサー設定は`ais_gng_node.ros__parameters`という既存の設定形式が対象。
