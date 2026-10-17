# 3D位置合わせの任意ツール

比較用の取得・入力変換・計測器。通常のCPU構築・回避実行への依存追加なし。
自作の補助コードだけを同梱、外部実装・重み・データは別配置。

リポジトリルートで:

```bash
bash extensions/registration/tools/clone_registration_repositories.sh /tmp/registration_eval
```

取得定義は[upstream.repos](upstream.repos)。新規取得は固定commit、既存checkoutは維持。
Python・CUDA等の環境は中核のDockerと分離。[導入・比較手順](../../gng_vlut_system/docs/REGISTRATION_EVALUATION.md)を参照。

現行Viewerの照合アダプタは中核側に維持。本ツールとの自動接続なし。
別Gitリポジトリへの切出し前の配置であり、独立した公開・リリースは未実施。
