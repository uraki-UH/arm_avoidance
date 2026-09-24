# メッシュモデルの直接表示

URDFやROSノードを追加せず、ローカルのメッシュを面付きで表示するTSX側の機能。
点群化・GNG学習・ROSへの配信は不要。既存の点群・グラフと同じ画面で比較可能。

## 操作

1. Viewerを再読み込みし、サイドバーの `Data → Mesh Models` を開く。
2. `ファイル選択` でモデルと付属ファイルを同時選択、または `フォルダ選択` でモデルのフォルダを選択。
3. 複数モデルがある場合はプルダウンで1つ選び、原本の単位を `m` / `cm → m` / `mm → m` から選択。
4. `追加` で読み込み。追加時はモデル全体へカメラを移動。
5. 位置・回転・拡大率の調整、表示切替、`全体を見る`、`削除` が可能。

位置はworld座標のm、回転の入力はdeg。単位倍率と表示上の拡大率は別扱い。
元の軸・原点・頂点座標を保持し、Y-upからZ-upへの変換や床位置への自動整列は行わない。
Y-upの原本をZ-upにする場合は、表示回転のXを+90 degに設定。
「明るさ補助」は追加メッシュだけに作用する補助発光。原本の材質・既存ロボットの照明は変更しない。

選択対象は**ブラウザを開いているPCのファイル**。Docker内のパス指定やサーバーへのアップロードではない。
表示と姿勢設定はページ内だけで保持し、再読み込みで解除。

## 対応形式

| 形式 | 表示内容・付属ファイル |
| --- | --- |
| OBJ | 面、MTLの材質、PNG/JPEGなどの画像。OBJ・MTL・画像を同時選択 |
| PLY | 面と頂点色。面を持たない点群PLYは対象外 |
| STL | binary / ASCIIの面 |
| GLB | 非圧縮glTF 2.0の内包メッシュ・材質 |
| glTF | JSONと参照先BIN・画像を同時選択 |
| FBX | Three.jsのFBXLoader対応データの静止メッシュ・材質 |

MTL・BIN・画像は選択ファイル内だけで解決。相対フォルダ構造を優先し、同名ファイルが曖昧ならエラー。
外部HTTP URLからの付属ファイル取得は行わない。
MTLが配布されていないOBJは警告を出して標準材質で表示。画像・BINの読込失敗は追加エラー。

## 取得済みモデルでの例

保存先は `/home/uraki/datasets/vehicle_models`。次のフォルダ内を選択。

- `poly_pizza/bicycle/bicycle.glb`: 小容量の自転車モデル。ただし座標上の最大寸法は約861.7で、既定のm扱いでは巨大化。`mm → m`で約0.862 m、表示回転Xを+90 degにするとZ-up。実寸の根拠は未確認。
- `mirageym/road_bike/RoadBike_SubDiv.fbx`: 詳細ロードバイク。
- `opengameart/fancy_motorcycle/bike.obj`: 軽量バイク。配布元にMTLがないため材質不足の警告あり。
- `artec3d/motorbike/obj/`: `Bike.obj`、`Bike.mtl`、`Bike_0.png`をまとめて選択。テクスチャ付きバイク。
- `artec3d/motorbike/ply/Motorbike_ply.ply`: バイクの面付きPLY。独自テクスチャ拡張は非対応のため、写真の色が必要な場合はOBJ版を使用。
- `artec3d/semi_trailer_truck/preview_300k/Truck_preview.glb`: 生成済みの軽量トラック。約44.8 MB、30万三角形、4K画像内包。単位は`m`、回転は0 degのままで表示可能。

## 大きなOBJの表示用GLB

トラックの原本は変更せず、次の単一ファイルをViewerへ追加。

```text
/home/uraki/datasets/vehicle_models/artec3d/semi_trailer_truck/preview_300k/Truck_preview.glb
```

1,500万面・約1.26 GBのOBJを、30万面・44,814,004 bytesのGLBへ変換済み。
8192×8192の画像は4096×4096へ縮小して内包。原本の軸・原点を保持し、表示座標だけ0.001倍。
Viewerでさらに`mm → m`を指定すると二重縮小になるため、`m`を選択。
寸法は約2.970×16.735×4.096 m。0.001倍は原本座標の桁とトラック寸法からの判断で、配布元の単位メタデータによる自動判定ではない。
Zの下端は約−0.0446 mで、床合わせは未適用。原点を動かす必要がある場合は表示位置を調整。

`preview.json`に原本ハッシュ・面数・寸法・倍率行列・近似誤差・変更内容を記録。
原本表面の面積比例サンプル2万点から軽量表面への片方向距離は、上記倍率で平均1.061 mm、rms 1.770 mm、サンプル内最大14.607 mm。
全表面の最大誤差や登録精度の保証ではなく、表示用の近似品質の確認値。
`SOURCE.json`と同梱`license.txt`も保持。配布時は帰属情報・変更内容をGLBと一緒に提供。

### 再生成

[生成ツール](../tools/prepare_mesh_preview.py)はオフライン処理。ROS・ブラウザの追加実行依存なし。
OBJのUVを考慮した面数削減後、GLBへ変換。既存の出力フォルダへの上書きは禁止。
以下はリポジトリルートでの再現例。再実行時は未使用の出力フォルダ名を指定。

```bash
mesh_preview_env=$(mktemp -d /tmp/mesh-preview-env-XXXXXX)
python3 -m venv --without-pip "$mesh_preview_env"
python3 -m pip --python "$mesh_preview_env" install -r ToPoFuzzy-Viewer/tools/mesh_preview_requirements.txt
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 "$mesh_preview_env/bin/python" \
  ToPoFuzzy-Viewer/tools/prepare_mesh_preview.py \
  /home/uraki/datasets/vehicle_models/artec3d/semi_trailer_truck/extracted/truck/Truck.obj \
  /home/uraki/datasets/vehicle_models/artec3d/semi_trailer_truck/preview_custom \
  --num_faces 300000 --max_texture_size 4096 --unit_scale 0.001 \
  --attribution_file /home/uraki/datasets/vehicle_models/artec3d/semi_trailer_truck/SOURCE.json
```

今回の変換は低優先度、メモリ上限18 GiB、30分タイムアウトで実行し、約314秒で完了。
同時実行する他の処理に合わせた空きメモリの確認が必要。一般のOBJ全形式への互換性は未保証。
頂点・UV・テクスチャを伴うOBJと無着色OBJの小規模テスト、今回の実トラックで検証。

軽量GLBのブラウザ検証（Frontendディレクトリ）：

```bash
MESH_PREVIEW=/home/uraki/datasets/vehicle_models/artec3d/semi_trailer_truck/preview_300k/Truck_preview.glb \
  node tests/local_mesh_browser.test.mjs
```

## 制限

- モデル本体256 MiB、選択ファイル合計512 MiBまで。大きなOBJは解析時に一時停止や多量のメモリ消費が起きるため、上限内でも軽量版を推奨。
- Artecのトラック原本 `Truck.obj` は約1.26 GBのため上限超過。上記の生成済み軽量GLBを使用。
- Maya `.mb`、Blender `.blend`、ZIPの直接読込は対象外。
- Draco / Meshopt / KTX2などの追加デコーダが必要な圧縮データ、アニメーション再生、PLY独自の外部テクスチャ拡張は対象外。
- 初期姿勢の静止表示。TF追従・サーバー共有・ROSトピック配信・座標の書き出しは未実装。
- メッシュの点群化が必要な場合は別機能の [完全表面テンプレート手順](../../gng_vlut_system/docs/MESH_SURFACE_TEMPLATE.md) を参照。

## ビルドと検証

既存の開発用Frontendはソース変更を反映。表示されない場合はページを再読み込み。
本番配信の場合はビルド成果物を反映。

```bash
docker exec gng_cpu_container bash -lc 'cd /ros2_ws/src/ToPoFuzzy-Viewer/frontend && npm run build'
```

検証用ブラウザは専用プロファイル・CDPパイプを使用し、成功・失敗ともテスト終了時に停止。

```bash
cd ToPoFuzzy-Viewer/frontend
npm run lint
VEHICLE_MODELS=/home/uraki/datasets/vehicle_models node --test tests/local_mesh_loader.test.mjs
VEHICLE_MODELS=/home/uraki/datasets/vehicle_models node tests/local_mesh_browser.test.mjs
```
