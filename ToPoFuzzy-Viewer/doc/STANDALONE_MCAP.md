# スタンドアロンHTMLのMCAP再生

## 位置づけ

`ToPo-FUZZY_Manipulation_v1.html`は、ROS 2を起動せずにMCAPファイルを直接選択し、
`sensor_msgs/msg/PointCloud2`と`ais_gng_msgs/msg/TopologicalMap`を時刻順に表示できる。
raw MCAPは原本確認用、bundle JSONは携帯端末への配布や繰り返し閲覧用の軽量形式という役割分担。

ブラウザへ組み込む処理はMCAP、CDR、ROS 2 message定義の復号のみ。GNG学習・推論アルゴリズム、
製品固有モデル、製品固有パラメータは含まない。GNGの再計算が必要な場合はROS 2バックエンドを使用する。

## 使用方法

1. `ToPo-FUZZY_Manipulation_v1.html`と`topo_mcap_reader.js`を同じ配布ルートへ配置
2. HTMLの「MCAP / ROS bag JSONを読み込む」から`.mcap`を選択
3. 対象topicを選択し、「再生」を選択

`file://`直開きに加え、静的HTTPサーバーからの配信を推奨。readerの再生成コマンドは次のとおり。

```bash
cd ToPoFuzzy-Viewer/frontend
npm ci
npm run build:mcap-reader
```

## 対応範囲

- MCAP profile: ROS 2
- schema encoding: `ros2msg`
- message encoding: `cdr`
- 圧縮: MCAP support libraryが扱うzstd、lz4、bz2
- 表示対象: `PointCloud2`、`TopologicalMap`
- 非対応: ROS 1 bag、ROS 2 sqlite3 `.db3`、任意message型、GNG再計算

同じtopic名を複数channelが使用する場合、2件目以降はtopic名へchannel IDを付けた別項目として表示する。

## 携帯端末向け制限

raw MCAPの全展開によるメモリ枯渇を避けるため、各対象topicは最大300フレーム、
PointCloud2は1 topicあたり合計約200万点を上限として時間・点を等間隔に間引く。
topic一覧には原本のmessage数、読込数、間引き幅を表示する。

この制限は可視化用であり、全messageの保存性を保証するものではない。全フレームの確認、巨大MCAP、
TFを用いた座標変換、編集、GNG再計算にはバックエンド再生を使用する。携帯端末へ渡す用途では、
必要topic・時間範囲・点数へ変換済みのbundle JSONを推奨する。

## 配布時の知財確認

readerの第三者依存はMIT、BSD-3-Clause、Apache-2.0、0BSD。製品配布時は
`third_party_licenses/topo_mcap_reader`全体を`topo_mcap_reader.js`と一緒に同梱する。
パッケージ更新時はesbuild metafileで実際に組み込まれた依存を再抽出し、
`THIRD_PARTY_NOTICES.md`とライセンス本文を更新する。
