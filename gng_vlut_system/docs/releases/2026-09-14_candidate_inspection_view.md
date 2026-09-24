# 2026-09-14 - 候補ノードの独立3Dビュー

## 1. 要約

Topo Fuzzy Viewerで候補ノードをクリックし、対象だけを別ウィンドウで回転・拡大縮小・平行移動する機能。
単体HTMLではなく`ToPoFuzzy-Viewer`が対象。

- 既存のクラスタ詳細パネルを独立カメラ・自動フィット・固定スナップショットの共通ビューへ変更。
- ノードの切り出しをブラウザから既存`viewer_edit_node`へ移管。

- `/grasp_pose_cands/nodes`の候補をクリックして開く操作。平面と付属非平面は同じ候補IDの部品として表示。
- 通常グラフのノード・クラスタ選択。所属の明示情報がないノードは単独表示。
- 「全体表示」「最新を取得」、ノード・エッジ・法線表示、ノード数・エッジ数・XYZ寸法・座標系の表示。
- ヘッダーのドラッグ移動、右下のサイズ変更。編集・領域指定モード中は選択無効。

- 別ソースにある同じクラスタIDの取り違えと、ノードID／エッジ添字の混同。
- カメラ操作をクリック選択と誤認する問題。移動判定は押下中のDOMイベントのみで、ポインター移動のための全ノードraycastは追加なし。

**削除**

- 詳細ビュー専用のノード・エッジ個別メッシュ描画。主画面の一括描画を再利用。

## 2. 条件・検証

1. Topicsで`/grasp_pose_cands/nodes`等をONにし、表示ノードを左クリック。
2. 独立ビュー内は左ドラッグで回転、ホイールで拡大縮小、右ドラッグで平行移動。
3. 形状は開いた時点で固定。「最新を取得」で更新し、「閉じる」で終了。

主画面のカメラ・TF・ROSデータへの変更なし。入力にないメッシュや接続の補完なし。
エッジ・法線情報を含まないMarkerには、それらの描画なし。寸法はノード中心群の範囲で、物体の確定外形ではない。

ROSトピック・launch引数・メッセージ定義の追加変更なし。WS v2に`edit.inspect_graph`を追加。
詳細は[API仕様](../../../ToPoFuzzy-Viewer/doc/BACKEND_API.md#候補の独立表示)を参照。

- Docker `/ros2_ws`で`colcon build --packages-select topo_fuzzy_viewer --symlink-install --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release`成功。
- `build/topo_fuzzy_viewer/test_graph_inspection`の5件成功。IDと添字、単独ノード、非平面所属、異常入力、Marker部品と姿勢変換を確認。
- frontendの`npm run lint`成功。`npm run build -- --configLoader runner --outDir /tmp/codex-candidate-inspection-test-dist`成功。Docker所有キャッシュへの権限問題を避けるためrunnerと一時出力先を使用。
- frontendコンテナ内の通常buildは既存MCAP依存5モジュール不足で失敗。ホスト側の既存依存で上記buildを確認。依存追加・既存コンテナ変更なし。
- ROS domain 225、WS port 19001の隔離テストと専用headless Chromeで、クリックから30ノードの別表示、独立回転・ズーム・平行移動・全体表示・固定保持・明示更新・ウィンドウ移動を検証。
- 大規模な実環境入力での負荷計測と実ユーザー画面での確認は未実施。

**制約**

- 使用にはビルド後の`viewer_stack.launch.py`再起動とブラウザ再読込が必要。作業中の既存Viewerは停止・再起動なし。
- 所属の複数候補、対象消失、取得失敗はエラー表示。既存スナップショットがある場合は保持。
- 抽出処理はクリック・明示更新時だけ。ただし受信フレーム全体のJSON転送があるため、大規模入力では取得待ちの可能性。
- 検証用の起動コマンド・停止結果は[進捗記録](../progress.md#2026-09-14-候補ノードの独立3dビュー)を参照。
