# 2026-09-14 - 平面クラスタの疎な接続と設定転送

採用判断: ユーザー評価により本改善案は不採用。[判断記録](../reject.md)を参照。以下は実施済み変更・検証の履歴であり、コード・設定の取り消しは未実施。

## 1. 要約

接続1本の隣接平面クラスタも幾何判定の対象とし、CPU版へ共通YAML設定を反映。

- `merge_connection_requirement`のROS既定値と同梱YAMLを2から1へ変更。ノード単体の`connection_requirement`は2のまま。
- クラスタ計算本体・残差上限・残差増加・少数側残差・面内広がり比の判定は変更なし。

- 疎な接続での同一平面統合と段差分離、launch設定転送・上書き順の回帰テスト。これらの追加テストは不採用後のユーザー依頼により削除済み。以下の検証結果は削除前の実施履歴。

- `ais_gng.launch.py`で`plane_params_file`の共通設定を`plane_cluster.*`へ転写。共通設定、`ais_gng_node`固有設定、起動引数の順で優先。
- 名前変更された`plane_cluster_visualization_node`にも共通設定を明示的に適用。

**削除**

- 非平面設定2項目だけを読み取る専用ヘルパー。トピック・メッセージの削除なし。

## 2. 条件・検証

- GNG接続1本だけのクラスタ対も、既存の幾何条件に適合した場合は統合可能。
- 既存の独自YAMLに書かれた統合設定もCPU版へ反映。従来無視されていた値による挙動変化に留意。
- CPU版の計算済みrho再利用は、同梱YAMLのCPU固有設定で維持。稼働中ノードへの自動適用なし。

- topic・message・launch引数の追加なし。従来の接続制限はYAMLの`merge_connection_requirement: 2`で指定可能。

- Docker内でCPUコンポーネント・平面ノードのビルド成功。C++22件とlaunch2件の回帰テスト成功。
- 実入力80フレームを隔離ドメイン225で同一順序再生。初期10フレームを除く70件で平均クラスタ数9.73から8.87、所属ノード数1718.24から1717.59。60件でクラスタ数減少。

Docker内、ROS環境読込後の検証コマンド:
```bash
cmake --build /ros2_ws/build/ais_gng --target ais_gng_component_cpu plane_cluster_incremental_node test_plane_cluster_incremental -j2
ctest --test-dir /ros2_ws/build/ais_gng -R '^test_plane_cluster_(incremental|launch)$' --output-on-failure
```

**制約**

- クラスタ数の減少は全箇所の物理的な同一平面性の証明ではない。Viewerでの修正後の目視確認は未実施。
- 残差増加や細長さによる過分割は今回の変更対象外。
- 一時比較スクリプトのROSノードは停止済み。既存GNG・Viewer・ROS daemonは再起動なし。画面への反映には既存GNG launchの再起動が必要。
