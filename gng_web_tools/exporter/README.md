# gng_bundle_exporter

ROS bagを単体HTML用のJSON／gzipへ変換。
ビルドは[親README](../README.md)、設定は[export_topics.example.yaml](config/export_topics.example.yaml)。

```bash
ros2 run gng_bundle_exporter gng-bundle-export export \
  --bag /path/to/bag \
  --config /ros2_ws/src/gng_web_tools/exporter/config/export_topics.example.yaml \
  --output /path/to/output.json.gz
```

topic確認は同CLIの`list --bag`。形式はtopicごとの`schema`・`rows`。
GNGのノードは`nodes`、目標・可操作性は`node_features`へ分離。
`compact`は関節角配列を省略するため、後続用途に必要かを確認。

出力先省略時はソース配置の`exporter/results/`へ時刻付き保存。既存ファイルは上書きせず別名を使用。
通常インストール時は配置に依存するため出力先を明示。
