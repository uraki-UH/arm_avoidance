# G3Reg初期検証

## 変更内容

- G3Reg公式実装へGUIなしの`headless_reg`実行器を追加する評価パッチを整備
- 既存GNG候補から作った対応点をG3Regの後段へ渡す`headless_corresp`評価器を追加
- 同梱Livox点群と、既存データセットのmug点群で処理時間と復元精度を確認

## 検証結果

同梱Livoxデータでは、G3Reg全体の総時間は約79 msだった。mug点群では、既知のZ軸0.5 rad回転と並進`(0.10, 0.02, 0.03)`を100対応点から正しく復元し、PAGORの総時間は約14.5 msだった。

対応点数を増やした場合の総時間は、10点11.0 ms、50点10.8 ms、100点14.5 ms、500点34.6 ms、1,000点171.4 msだった。数十〜数百件に抑えれば、CPU後段として利用できる見込みがある。

## 適用範囲と未完了事項

現在の物体PCDをG3Regの生点群フロントエンドへ直接入力すると、物体サイズ向けにパラメータ調整しても前処理中に異常終了した。G3Reg固有のLiDAR前処理を現行の物体点群へそのまま適用するのは保留し、既存の平面クラスタ・非平面成分から対応を生成して後段へ渡す方針とする。

対応生成の誤り、遮蔽、部分観測、実フレームでの候補再現率・誤検出率は未評価である。したがって、G3Regは現時点では本番の物体召喚経路へ接続していない。

## 再現用ファイル

- `gng_vlut_system/tools/registration_eval/g3reg_headless.patch`
- `gng_vlut_system/tools/registration_eval/g3reg_correspondence.patch`
- `gng_vlut_system/tools/registration_eval/g3reg_correspondence.cpp`
