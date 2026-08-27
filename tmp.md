ais_gngについてすでにこの中にボクセル化処理があるので、world_indexbucketの処理とかでもボクセルに点群登録している処理が重複している気がしていてどうにかできる？

できます。ただし ais_gng の内部VoxelGridをそのまま world_index_bucket に流用するのは危険です。
現時点の実行中Dockerには world_index_to_voxel_node がいないため、今この起動構成では重複は発生していません。
役割が異なります。
- ais_gng のVoxelGrid: GNG学習用のフレーム内・固定格子で、各セルを平均化して入力点を減らす処理。[voxel_grid.cpp (line 21)](/home/fuzzrobo/uraki_ws/ais_gng_cpu/src/gng_cpu/src/cpu/voxel_grid.cpp#L21)
- world_index_bucket: world座標へ変換した元点群を粗いbucketへ登録し、複数ロボットROIごとに必要な点だけ取り出してから衝突用voxel IDへ変換する処理。[world_index_to_voxel_node.cpp (line 619)](/home/fuzzrobo/uraki_ws/gng_vlut_system/src/nodes/bridge/world_index_to_voxel_node.cpp#L619)
GNG用の平均点だけをworld indexに渡すと、ROI・衝突判定で必要な細部が失われます。またGNG内部の格子はprivateなフレーム内バッファで、world indexはworld座標・複数ROI・別解像度を前提にします。
推奨は、内部Gridを共有するのでなく、入力直後に共有の階層点群インデックスを一回だけ作る方式です。
1. PointCloudをworld座標へ一回だけ変換
2. 同じ走査で粗いworld bucketと、GNG用の細かいvoxelセルを同時構築
3. world indexは各bucketに元点への参照を保持
4. GNGは細かいセルの重心と、元点ID範囲を受け取る
5. ROI側は粗いbucketから元点を取り出し、必要なROIだけ最終voxel化
bucket_size=0.2 とGNGの input.voxel_grid_unit=0.02 のように整数倍で揃えると、粗いbucketを親、GNG voxelを子にした階層にできます。
短期的には ais_gng の /scan/transformed をworld indexの入力にしてTF変換の重複だけ減らせます。ただしvoxel化自体は二重のままです。
完全統合では、GNGへ「事前voxel化済み入力」を渡す狭いAPIを追加して、内部VoxelGridをbypassするのが保守性も良いです。

現状では、そのケースを明示的な平面境界としては扱っていません。
現在の rho の用途は、新規クラスタ生成時に平坦なノードを種として優先するだけです。[plane_cluster_incremental.cpp (line 498)](/home/fuzzrobo/uraki_ws/ais_gng_cpu/src/ais_gng/src/topological_plane/plane_cluster_incremental.cpp:498)
そのため、
- rho が急変していても
- ノード位置が同一平面に収まり
- 法線角度と残差条件を通り
- GNG edgeで接続されている
場合は、同じ平面クラスタへ取り込まれます。さらに分離しても、結合後の平面性が良ければ併合処理で再結合する可能性があります。
これは平面境界を残したい場合には不足しています。適切なのは、高 rho ノードを除外するのではなく、境界を越える伝播だけ止める方式です。
平面内部 ─ 低rho ─ 高rho境界 ─ 別領域
具体的には次の扱いがよいです。
- 高 rho ノード自身は平面クラスタに所属可能
- 高 rho ノードを経由して、その先のノードを取り込まない
- rho の急変edgeを取り込み・移動の接続本数に数えない
- 分割時は急変edgeを接続として扱わない
- 併合時も急変edgeを併合根拠に数えない
- 数フレーム継続した場合だけ境界確定する
これなら境界部分の点を失わず、床から曲面、平面から側面などへのクラスタ漏れを止められます。現状は「平面性が維持されるなら同一面を優先」なので、rh


ガウシアンミクスチャーモデルみたいなの使える？なんかガウシアンスプラっティングみたいだけどね







9:32 PM






使えます。むしろ現在のGNG nodeは、GMMの成分にかなり自然に対応します。
\[
p(x)=\sum_i \pi_i \mathcal{N}(x;\mu_i,\Sigma_i)
\]- 平均 μ_i: GNG node位置
- 共分散 Σ_i: 勝者点群共分散を基礎にする
- 重み π_i: 勝者点数、または局所表面面積に基づく重み
ただし、勝者点群共分散だけをそのまま使うのは不安定です。GNG密度や学習状態で小さくなり過ぎるので、照合用には
\[
\Sigma_i^{match} =
\Sigma_i^{winner}
+ \Sigma_i^{kernel}
+ \Sigma_i^{sensor}
\]のように、物理スケールで決めた最小カーネルを足すのがよいです。
- 接平面方向: 少し広めの許容幅
- 法線方向: 深度ノイズ・表面厚み用の小さい許容幅
- 固有値に下限を設ける
- テンプレート全体で一つのスケール倍率範囲を持たせる
欠損に強い照合
環境側の成分 j をテンプレートGMMに片方向で評価します。
\[
S =
\sum_j w_j
\log \left(
\sum_i \pi_i
\mathcal{N}(
\mu_j;\,
\mu_i,\,
\Sigma_i+\Sigma_j
)
\right)
\]これなら、環境で観測できた面がテンプレートのどこかに説明されれば高得点になります。見えていないテンプレート側の面は罰しないので、遮蔽・欠損に対応しやすいです。
yawだけ見るなら、各yaw候補でテンプレートの平均・共分散を R_z で回します。
\[
\mu_i' = R_z\mu_i,\quad
\Sigma_i' = R_z\Sigma_iR_z^\top
\]平行移動を推定しない方針なら、各クラスタを自分の重心や基準node中心へ正規化してから評価します。これは位置合わせの探索ではなく、座標原点を消す前処理です。
Gaussian Splattingとの違い
- 3D Gaussian Splatting: 画像レンダリング向け。色・不透明度・視点依存表現を持つ
- 今回のGMM: 3D空間上の確率密度・形状照合向け。色や画像レンダリングは不要
Viewerでは同じ楕円体を表示できますが、照合本体は画像化せずガウス尤度を直接計算する方が軽くて保守しやすいです。
ただしGMMだけではedge接続や部位構造が弱いので、最終スコアは「GMM尤度 + 法線/edge不変特徴 + 階層ボール整合性」にするのがよいです。