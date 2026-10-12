# lib_dynamixel

Robotis社の[Dynamixel](https://e-shop.robotis.co.jp/list.php?c_id=89)をROSから制御するためのライブラリです．  
基本的にROSで使うことを想定しているので，単体でのビルドは試してない．
ROS1に組み込んだものが[こちら](https://github.com/SHINOBI-organization/DynamixelHandler-ros1/tree/main)
ROS2に組み込んだものが[こちら](https://github.com/ROBOTIS-JAPAN-GIT/DynamixelHandler-ros2/tree/main)

## API List

短く書くために，型名と引数を略記しています．

**クラスの設定**
- `DynamixelCommunicator` [`DynamixelCommunicator()`](#api-ctor-default)
- `DynamixelCommunicator` [`DynamixelCommunicator(port_name, baudrate, latency_timer = 16)`](#api-ctor-port)
- `bool` [`OpenPort()`](#api-open-port)
- `bool` [`ClosePort()`](#api-close-port)
- `void` [`set_baudrate(baudrate)`](#api-set-baudrate)
- `void` [`set_latency_timer(latency_timer)`](#api-set-latency-timer)
- `void` [`set_status_return_level(level)`](#api-set-status-return-level)
- `void` [`set_verbose(verbose)`](#api-set-verbose)
- `void` [`set_retry_config(num_try, msec_interval)`](#api-set-retry-config)
- `string` [`port_name()`](#api-port-name)
- `void` [`GetPortHandler(port_name)`](#api-get-port-handler)

**Dynamixelとの基本通信**
- `void` [`Reboot(id_t)`](#api-reboot)
- `void` [`FactoryReset(id_t, level)`](#api-factory-reset)
- `bool` [`Ping(id_t)`](#api-ping)
- `bool` [`tryPing(id_t)`](#api-try-ping)
- `vector<id_t>` [`Ping_broadcast()`](#api-ping-broadcast)

**Dynamixelへの単体書き込み**
- `bool` [`Write(address, id_t, data_t)`](#api-write-single)
- `bool` [`tryWrite(address, id_t, data_t)`](#api-try-write-single)
- `bool` [`Write(vector<address>, id_t, vector<data_t>)`](#api-write-multi)
- `bool` [`tryWrite(vector<address>, id_t, vector<data_t>)`](#api-try-write-multi)

**Dynamixelとの一括書き込み**
- `bool` [`SyncWrite(address, map<id_t, data_t>)`](#api-syncwrite-map-single)
- `bool` [`SyncWrite(vector<address>, map<id_t, vector<data_t>>)`](#api-syncwrite-map-multi)
- `bool` [`SyncWrite(address, vector<id_t>, vector<data_t>)`](#api-syncwrite-vec-single) (非推奨)
- `bool` [`SyncWrite(vector<address>, vector<id_t>, vector<vector<data_t>>)`](#api-syncwrite-vec-multi) (非推奨)
- `bool` [`BulkWrite(map<id_t, address>, map<id_t, data_t>)`](#api-bulkwrite-map-single)
- `bool` [`BulkWrite(map<id_t, vector<address>>, map<id_t, vector<data_t>>)`](#api-bulkwrite-map-multi)
- `bool` [`BulkWrite(vector<address>, vector<id_t>, vector<data_t>)`](#api-bulkwrite-vec-single) (非推奨)
- `bool` [`BulkWrite(vector<vector<address>>, vector<id_t>, vector<vector<data_t>>)`](#api-bulkwrite-vec-multi) (非推奨)

**Dynamixelへの単体読み出し**
- `data_t` [`Read(address, id_t)`](#api-read-single)
- `data_t` [`tryRead(address, id_t)`](#api-try-read-single)
- `vector<data_t>` [`Read(vector<address>, id_t)`](#api-read-multi)
- `vector<data_t>` [`tryRead(vector<address>, id_t)`](#api-try-read-multi)

**Dynamixelとの一括読み出し**
- `map<id_t, data_t>` [`SyncRead(address, vector<id_t>)`](#api-syncread-single)
- `map<id_t, data_t>` [`SyncRead_fast(address, vector<id_t>)`](#api-fast-read-apis)
- `map<id_t, vector<data_t>>` [`SyncRead(vector<address>, vector<id_t>)`](#api-syncread-multi)
- `map<id_t, vector<data_t>>` [`SyncRead_fast(vector<address>, vector<id_t>)`](#api-fast-read-apis)
- `map<id_t, data_t>` [`BulkRead(map<id_t, address>)`](#api-bulkread-single)
- `map<id_t, data_t>` [`BulkRead_fast(map<id_t, address>)`](#api-fast-read-apis)
- `map<id_t, vector<data_t>>` [`BulkRead(map<id_t, vector<address>>)`](#api-bulkread-multi)
- `map<id_t, vector<data_t>>` [`BulkRead_fast(map<id_t, vector<address>>)`](#api-fast-read-apis)

**クラス変数からの情報取得**
- `bool` [`timeout_last_read()`](#api-timeout-last-read)
- `bool` [`comm_error_last_read()`](#api-comm-error-last-read)
- `bool` [`hardware_error_last_read()`](#api-hardware-error-last-read)
- `vector<id_t>&` [`hardware_error_id_last_read()`](#api-hardware-error-id-last-read)
- `map<id_t, uint16_t>&` [`ping_id_model_map_last_read()`](#api-ping-id-model-map-last-read)
- `double` [`dead_time_retry_ping()`](#api-dead-time-retry-ping)
- `double` [`wait_time_ping_broadcast()`](#api-wait-time-ping-broadcast)

## Address Map
`src/dynamixel_parameters.h` に記載のあるパラメータ/アドレスが，現在の対応対象です．

## API の 詳細説明

<a id="api-ctor-default"></a>
### `DynamixelCommunicator()`

戻り値:
なし（コンストラクタ）．

<a id="api-ctor-port"></a>
### `DynamixelCommunicator(const char *port_name, int baudrate, int latency_timer = 16)`
`port_name` に接続先デバイス名を，`baudrate` に通信速度を，`latency_timer` にレイテンシタイマーを指定して初期化します．

戻り値:
なし（コンストラクタ）．
```cpp
DynamixelCommunicator dxl("/dev/ttyUSB0", 1000000, 16);
```

<a id="api-open-port"></a>
### `OpenPort()`
ポートを開き，レイテンシタイマーおよびボーレートを設定します．

戻り値:
`bool`．ポートオープンとボーレート設定に成功したとき `true`，失敗時 `false` を返します．
```cpp
DynamixelCommunicator dxl("/dev/ttyUSB0", 1000000, 16); // 通信の設定
if (dxl.OpenPort()) {
    // ポートが正常に開かれた場合の処理
}
```

<a id="api-close-port"></a>
### `ClosePort()`
ポートを閉じます．

戻り値:
`bool`．現在の実装では常に `true` を返します．
```cpp
dxl.ClosePort();
```

<a id="api-set-baudrate"></a>
### `set_baudrate(int baudrate)`
通信速度を設定します．

戻り値:
`void`（返り値なし）．
```cpp
dxl.set_baudrate(57600);
```

<a id="api-set-latency-timer"></a>
### `set_latency_timer(uint8_t latency_timer)`
USBシリアル通信のレイテンシタイマーを設定します．

戻り値:
`void`（返り値なし）．
```cpp
dxl.set_latency_timer(8);
```

<a id="api-set-status-return-level"></a>
### `set_status_return_level(int level)`
ステータスパケットの返りを指定します．

戻り値:
`void`（返り値なし）．
```cpp
dxl.set_status_return_level(2);
```

<a id="api-set-verbose"></a>
### `set_verbose(bool verbose)`
動作ログを標準出力に表示するかを指定します．

戻り値:
`void`（返り値なし）．
```cpp
dxl.set_verbose(true);
```

<a id="api-set-retry-config"></a>
### `set_retry_config(int num_try, int msec_interval)`
通信失敗時のリトライ回数とリトライ間隔を設定します．

戻り値:
`void`（返り値なし）．
```cpp
dxl.set_retry_config(3, 50); // 3回まで失敗しても50ms間隔で再試行
```

<a id="api-port-name"></a>
### `port_name()`
使用しているポート名を取得します．

戻り値:
`string`．設定されているデバイス名（例: `/dev/ttyUSB0`）を返します．
```cpp
std::string name = dxl.port_name();
```

<a id="api-get-port-handler"></a>
### `GetPortHandler(const char *port_name)`
使用するシリアルポートハンドラを作成・設定します．  
通常はコンストラクタで設定されるため、明示的に差し替えたい場合に使用します．

戻り値:
`void`（返り値なし）．
```cpp
dxl.GetPortHandler("/dev/ttyUSB1");
```

<a id="api-reboot"></a>
### `Reboot(uint8_t servo_id)`
指定したDynamixelをリブートします．

戻り値:
`void`（返り値なし）．
```cpp
dxl.Reboot(1); // ID 1のサーボをリブート
```

<a id="api-factory-reset"></a>
### `FactoryReset(uint8_t servo_id, FactoryResetLevel level)`
指定したDynamixelを工場出荷状態に戻します．

戻り値:
`void`（返り値なし）．
```cpp
dxl.FactoryReset(1, FACTORY_RESET_EXCLUDE_ID_BAUDRATE); // ID 1のサーボをファクトリーリセット（IDとボーレートを除く）
```
FactoryResetLevel は以下のように定義されています．
  - `FACTORY_RESET_EXCLUDE_ID`         : ID以外のコントロールテーブルの値をリセット 
  - `FACTORY_RESET_EXCLUDE_ID_BAUDRATE`: IDとボーレート以外のコントロールテーブルの値をリセット
  - `FACTORY_RESET_ALL`                : 全てのコントロールテーブルの値をリセット

<a id="api-ping"></a>
### `Ping(uint8_t servo_id)`
指定したDynamixelに ping を送り，応答があるかを確認します．
応答があった場合，取得した `ID -> model_number` は `ping_id_model_map_last_read()` で参照できます．
この API では Broadcast ID (`0xFE`) は指定しません（単一ID向けです）．  
一斉探索は `Ping_broadcast()` を使用してください．

戻り値:
`bool`．応答が得られたとき `true`，失敗時 `false` を返します．
```cpp
bool is_alive = dxl.Ping(1); // ID 1のサーボにping
```

<a id="api-ping-broadcast"></a>
### `Ping_broadcast()`
Broadcast ID に ping を送り，応答したサーボを収集します．  
取得した `ID -> model_number` は `ping_id_model_map_last_read()` で参照できます．

戻り値:
`vector<uint8_t>`．応答したIDの配列を返します（0件なら空配列）．
```cpp
vector<uint8_t> found_ids = dxl.Ping_broadcast();
map<uint8_t, uint16_t>& id_model = dxl.ping_id_model_map_last_read();
```

<a id="api-try-ping"></a>
### `tryPing(uint8_t servo_id)`
指定したDynamixelに対して，応答があるか，繰り返し上限に達するまで ping を送ります．

戻り値:
`bool`．リトライ中に1回でも成功すれば `true`，全試行失敗で `false` を返します．
```cpp
bool is_alive = dxl.tryPing(1); // ID 1のサーボにpingを複数回試行
```

<a id="api-write-single"></a>
### `Write(DynamixelAddress address, uint8_t servo_id, int64_t data)`
Dynamixelに情報を書き込みます．

戻り値:
`bool`．書き込みコマンドが正常完了したとき `true`，失敗時 `false` を返します．
```cpp
DynamixelAddress addr(30, TYPE_UINT8);
dxl.Write(addr, 1, 255); // ID 1のサーボのアドレス30に255を書き込む
```

<a id="api-try-write-single"></a>
### `tryWrite(DynamixelAddress address, uint8_t servo_id, int64_t data)`
Dynamixelに情報を書き込み，正常応答があるか，繰り返し上限に達するまで試行します．

戻り値:
`bool`．リトライ中に1回でも成功すれば `true`，全試行失敗で `false` を返します．
```cpp
DynamixelAddress addr(30, TYPE_UINT8);
dxl.tryWrite(addr, 1, 255); // ID 1のサーボのアドレス30に255を書き込む（複数回試行）
```

<a id="api-write-multi"></a>
### `Write(const vector<DynamixelAddress>& address_vec, uint8_t servo_id, const vector<int64_t>& data_vec)`
複数のアドレスに情報を書き込みます．

戻り値:
`bool`．書き込みコマンドが正常完了したとき `true`，失敗時 `false` を返します．
```cpp
vector<DynamixelAddress> addrs = {
    DynamixelAddress(30, TYPE_UINT8), 
    DynamixelAddress(31, TYPE_UINT8)
};
vector<int64_t> data = {255, 128};
dxl.Write(addrs, 1, data); // ID 1のサーボのアドレス30と31にそれぞれ255と128を書き込む
```

<a id="api-try-write-multi"></a>
### `tryWrite(const vector<DynamixelAddress>& address_vec, uint8_t servo_id, const vector<int64_t>& data_vec)`
複数のアドレスに情報を書き込み，正常応答があるか，繰り返し上限に達するまで試行します．

戻り値:
`bool`．リトライ中に1回でも成功すれば `true`，全試行失敗で `false` を返します．
```cpp
vector<DynamixelAddress> addrs = {
    DynamixelAddress(30, TYPE_UINT8), 
    DynamixelAddress(31, TYPE_UINT8)
};
vector<int64_t> data = {255, 128};
dxl.tryWrite(addrs, 1, data); // ID 1のサーボのアドレス30と31にそれぞれ255と128を書き込む（複数回試行）
```

<a id="api-syncwrite-map-single"></a>
### `SyncWrite(DynamixelAddress dp, const map<uint8_t, int64_t>& id_data_int_map)`
複数のDynamixelの同一のアドレスに情報を書き込みます．

戻り値:
`bool`．書き込みコマンド送信に成功したとき `true`，失敗時 `false` を返します．
```cpp
DynamixelAddress addr(30, TYPE_UINT8);
map<uint8_t, int64_t> id_data = {{1, 255}, {2, 128}, {3, 64}};
dxl.SyncWrite(addr, id_data); // ID 1, 2, 3のサーボのアドレス30にそれぞれ255, 128, 64を書き込む
```

<a id="api-syncwrite-map-multi"></a>
### `SyncWrite(const vector<DynamixelAddress>& dp_list, const map<uint8_t, vector<int64_t>>& id_data_int_map)`
複数のDynamixelの複数のアドレスに情報を書き込みます．

戻り値:
`bool`．書き込みコマンド送信に成功したとき `true`，失敗時 `false` を返します．
```cpp
vector<DynamixelAddress> addrs = {
    DynamixelAddress(30, TYPE_UINT8), 
    DynamixelAddress(31, TYPE_UINT8)
};
map<uint8_t, vector<int64_t>> id_data = {{1, {255, 128}}, {2, {128, 64}}, {3, {64, 32}}};
dxl.SyncWrite(addrs, id_data); // ID 1, 2, 3のサーボのアドレス30と31にそれぞれ255, 128, 128, 64, 64, 32を書き込む
```

<a id="api-syncwrite-vec-single"></a>
### `SyncWrite(DynamixelAddress dp, const vector<uint8_t>& servo_id_list, const vector<int64_t>& data_int_list)` (非推奨)
複数のDynamixelの同一のアドレスに情報を書き込みます．

戻り値:
`bool`．書き込みコマンド送信に成功したとき `true`，失敗時 `false` を返します．
```cpp
vector<uint8_t> ids = {1, 2, 3};
vector<int64_t> data = {255, 128, 64};
DynamixelAddress addr(30, TYPE_UINT8);
dxl.SyncWrite(addr, ids, data); // ID 1, 2, 3のサーボのアドレス30にそれぞれ255, 128, 64を書き込む
```

<a id="api-syncwrite-vec-multi"></a>
### `SyncWrite(const vector<DynamixelAddress>& dp_list, const vector<uint8_t>& servo_id_list, const vector<vector<int64_t>>& data_vec_list)` (非推奨)
複数のDynamixelの複数のアドレスに情報を書き込みます．

戻り値:
`bool`．書き込みコマンド送信に成功したとき `true`，失敗時 `false` を返します．
```cpp
vector<DynamixelAddress> addrs = {
    DynamixelAddress(30, TYPE_UINT8), 
    DynamixelAddress(31, TYPE_UINT8)
};
vector<uint8_t> ids = {1, 2, 3};
vector<vector<int64_t>> data = {{255, 128}, {128, 64}, {64, 32}};
dxl.SyncWrite(addrs, ids, data); // ID 1, 2, 3のサーボのアドレス30と31にそれぞれ255, 128, 128, 64, 64, 32を書き込む
```

<a id="api-bulkwrite-map-single"></a>
### `BulkWrite(const map<uint8_t, DynamixelAddress>& id_dp_map, const map<uint8_t, int64_t>& id_data_int_map)`
複数サーボを対象に、各サーボへ1つずつ指定した（サーボごとに異なってよい）アドレスへ一括で書き込みます．
内部では `BulkWrite(const vector<DynamixelAddress>&, const vector<uint8_t>&, const vector<int64_t>&)` へ変換して処理します．

戻り値:
`bool`．引数チェックを通過して送信できたとき `true`，失敗時 `false` を返します．
```cpp
map<uint8_t, DynamixelAddress> id_addr_map = {
    {1, DynamixelAddress(116, TYPE_INT32)},
    {2, DynamixelAddress(102, TYPE_INT16)}
};
map<uint8_t, int64_t> id_data = {{1, 1024}, {2, 80}};
dxl.BulkWrite(id_addr_map, id_data); // ID 1のアドレス116に1024，ID 2のアドレス102に80を書き込む
```

<a id="api-bulkwrite-map-multi"></a>
### `BulkWrite(const map<uint8_t, vector<DynamixelAddress>>& id_dp_list_map, const map<uint8_t, vector<int64_t>>& id_data_vec_map)`
サーボごとに複数アドレスへ書き込みます．
各サーボ内のアドレス列は連続している必要があります．
内部では `BulkWrite(const vector<vector<DynamixelAddress>>&, const vector<uint8_t>&, const vector<vector<int64_t>>&)` へ変換して処理します．

戻り値:
`bool`．引数チェックを通過して送信できたとき `true`，失敗時 `false` を返します．
```cpp
map<uint8_t, vector<DynamixelAddress>> id_addrs_map = {
    {1, {DynamixelAddress(116, TYPE_INT32), DynamixelAddress(120, TYPE_INT16)}},
    {2, {DynamixelAddress(102, TYPE_INT16)}}
};
map<uint8_t, vector<int64_t>> id_data = {
    {1, {1024, 50}},
    {2, {80}}
};
dxl.BulkWrite(id_addrs_map, id_data); // ID 1はアドレス116/120に1024/50，ID 2はアドレス102に80を書き込む
```

<a id="api-bulkwrite-vec-single"></a>
### `BulkWrite(const vector<DynamixelAddress>& dp_list, const vector<uint8_t>& servo_id_list, const vector<int64_t>& data_int_list)` (非推奨)
複数サーボを対象に、各サーボへ1つずつ（サーボごとに異なってよい）アドレスへ一括で書き込みます．  
内部では `vector<vector<DynamixelAddress>>` 版へ変換して処理します．

戻り値:
`bool`．送信できたとき `true`，失敗時 `false` を返します．
```cpp
vector<DynamixelAddress> addrs = {
    DynamixelAddress(116, TYPE_INT32),
    DynamixelAddress(102, TYPE_INT16)
};
vector<uint8_t> ids = {1, 2};
vector<int64_t> data = {1024, 80};
dxl.BulkWrite(addrs, ids, data); // ID 1のアドレス116に1024，ID 2のアドレス102に80を書き込む
```

<a id="api-bulkwrite-vec-multi"></a>
### `BulkWrite(const vector<vector<DynamixelAddress>>& dp_lists, const vector<uint8_t>& servo_id_list, const vector<vector<int64_t>>& data_vec_list)` (非推奨)
サーボごとに異なる複数アドレスへ書き込みます．  
各サーボ内のアドレス列は連続している必要があります．

戻り値:
`bool`．送信できたとき `true`，失敗時 `false` を返します．
```cpp
vector<vector<DynamixelAddress>> dp_lists = {
    {DynamixelAddress(116, TYPE_INT32), DynamixelAddress(120, TYPE_INT16)},
    {DynamixelAddress(102, TYPE_INT16)}
};
vector<uint8_t> ids = {1, 2};
vector<vector<int64_t>> data = {{1024, 50}, {80}};
dxl.BulkWrite(dp_lists, ids, data); // ID 1はアドレス116/120に1024/50，ID 2はアドレス102に80を書き込む
```

<a id="api-read-single"></a>
### `Read(DynamixelAddress dp, uint8_t servo_id)`
Dynamixelから情報を読み込みます．

戻り値:
`int64_t`．読み込み成功時はデータ値，失敗時は `0` を返します．
```cpp
DynamixelAddress addr(30, TYPE_UINT8);
int64_t data = dxl.Read(addr, 1); // ID 1のサーボのアドレス30からデータを読み込む
```

<a id="api-try-read-single"></a>
### `tryRead(DynamixelAddress dp, uint8_t servo_id)`
Dynamixelから情報を読み込み，正常応答があるか，繰り返し上限に達するまで試行します．

戻り値:
`int64_t`．成功時はデータ値，すべて失敗した場合は最終試行の値（多くは `0`）を返します．
```cpp
DynamixelAddress addr(30, TYPE_UINT8);
int64_t data = dxl.tryRead(addr, 1); // ID 1のサーボのアドレス30からデータを読み込む（複数回試行）
```

<a id="api-read-multi"></a>
### `Read(const vector<DynamixelAddress>& dp_list, uint8_t servo_id)`
複数のアドレスから情報を読み込みます．

戻り値:
`vector<int64_t>`．成功時は各アドレスの値，失敗時は同じ要素数の `0` 埋め配列を返します．
```cpp
vector<DynamixelAddress> addrs = {
    DynamixelAddress(30, TYPE_UINT8), 
    DynamixelAddress(31, TYPE_UINT8)
};
vector<int64_t> data = dxl.Read(addrs, 1); // ID 1のサーボのアドレス30と31からデータを読み込む
```

<a id="api-try-read-multi"></a>
### `tryRead(const vector<DynamixelAddress>& dp_list, uint8_t servo_id)`
Dynamixelから情報を読み込み，正常応答があるか，繰り返し上限に達するまで試行します．

戻り値:
`vector<int64_t>`．成功時は各アドレスの値，すべて失敗した場合は最終試行の結果（多くは `0` 埋め）を返します．
```cpp
vector<DynamixelAddress> addrs = {
    DynamixelAddress(30, TYPE_UINT8), 
    DynamixelAddress(31, TYPE_UINT8)
};
vector<int64_t> data = dxl.tryRead(addrs, 1); // ID 1のサーボのアドレス30と31からデータを読み込む（複数回試行）
```

<a id="api-syncread-single"></a>
### `SyncRead(DynamixelAddress dp, const vector<uint8_t>& servo_id_list)`
複数のDynamixelの同一のアドレスから情報を読み込みます．

戻り値:
`map<uint8_t, int64_t>`．受信・検証に成功したIDのみ `ID -> データ` を返します（失敗IDは含まれません）．
```cpp
vector<uint8_t> ids = {1, 2, 3};
DynamixelAddress addr(30, TYPE_UINT8);
map<uint8_t, int64_t> data = dxl.SyncRead(addr, ids); // ID 1, 2, 3のサーボのアドレス30からデータを読み込む
```

<a id="api-syncread-multi"></a>
### `SyncRead(const vector<DynamixelAddress>& dp_list, const vector<uint8_t>& servo_id_list)`
複数のDynamixelの複数のアドレスから情報を読み込みます．

戻り値:
`map<uint8_t, vector<int64_t>>`．受信・検証に成功したIDのみ `ID -> データ配列` を返します．
```cpp
vector<DynamixelAddress> addrs = {
    DynamixelAddress(30, TYPE_UINT8),
    DynamixelAddress(31, TYPE_UINT8)
};
vector<uint8_t> ids = {1, 2, 3};
map<uint8_t, vector<int64_t>> data = dxl.SyncRead(addrs, ids); // ID 1, 2, 3のサーボのアドレス30と31からデータを読み込む
```

<a id="api-bulkread-single"></a>
### `BulkRead(const map<uint8_t, DynamixelAddress>& id_dp_map)`
複数サーボを対象に、各サーボへ1つずつ指定した（サーボごとに異なってよい）アドレスを一括で読み込みます．

戻り値:
`map<uint8_t, int64_t>`．受信・検証に成功したIDのみ `ID -> データ` を返します．
```cpp
map<uint8_t, DynamixelAddress> id_addr_map = {
    {1, DynamixelAddress(132, TYPE_INT32)},
    {2, DynamixelAddress(126, TYPE_INT16)}
};
map<uint8_t, int64_t> id_data = dxl.BulkRead(id_addr_map); // ID 1はアドレス132，ID 2はアドレス126を読み込む
```

<a id="api-bulkread-multi"></a>
### `BulkRead(const map<uint8_t, vector<DynamixelAddress>>& id_dp_list_map)`
サーボごとに異なる複数アドレスを読み込みます．  
各IDでアドレス列の内容や要素数を変えられます（これが SyncRead との主な違いです）．

戻り値:
`map<uint8_t, vector<int64_t>>`．受信・検証に成功したIDのみ `ID -> データ配列` を返します．
```cpp
map<uint8_t, vector<DynamixelAddress>> id_addrs_map = {
    {1, {DynamixelAddress(132, TYPE_INT32), DynamixelAddress(126, TYPE_INT16)}},
    {2, {DynamixelAddress(116, TYPE_INT32)}}
};
map<uint8_t, vector<int64_t>> id_data = dxl.BulkRead(id_addrs_map); // ID 1はアドレス132/126，ID 2はアドレス116を読み込む
```

<a id="api-fast-read-apis"></a>
### `BulkRead_fast(...) / SyncRead_fast(...)`
高速版の API です．基本の返り値・引数形式は通常版と同じです．

戻り値:
`BulkRead_fast(map<id_t, address>)` は `map<id_t, data_t>`，  
`BulkRead_fast(map<id_t, vector<address>>)` は `map<id_t, vector<data_t>>`，  
`SyncRead_fast(address, vector<id_t>)` は `map<id_t, data_t>`，  
`SyncRead_fast(vector<address>, vector<id_t>)` は `map<id_t, vector<data_t>>` を返します．

<a id="api-timeout-last-read"></a>
### `timeout_last_read()`
直近の通信がタイムアウトで失敗したかを返します．

戻り値:
`bool`．タイムアウト発生時 `true`，それ以外 `false` です．
```cpp
bool timeout = dxl.timeout_last_read();
```

<a id="api-comm-error-last-read"></a>
### `comm_error_last_read()`
直近の通信で通信エラーが発生したかを返します．

戻り値:
`bool`．通信エラー発生時 `true`，それ以外 `false` です．
```cpp
bool comm_err = dxl.comm_error_last_read();
```

<a id="api-hardware-error-last-read"></a>
### `hardware_error_last_read()`
直近の通信でハードウェアエラーが発生したかを返します．

戻り値:
`bool`．ハードウェアエラー発生時 `true`，それ以外 `false` です．
```cpp
bool hw_err = dxl.hardware_error_last_read();
```

<a id="api-hardware-error-id-last-read"></a>
### `hardware_error_id_last_read()`
直近の通信でハードウェアエラーが発生したサーボIDの一覧を返します．

戻り値:
`vector<id_t>&`（実体は `vector<uint8_t>&`）．ハードウェアエラーが出たID一覧への参照を返します．
```cpp
vector<uint8_t>& err_ids = dxl.hardware_error_id_last_read();
```

<a id="api-ping-id-model-map-last-read"></a>
### `ping_id_model_map_last_read()`
直近の `Ping()` / `Ping_broadcast()` で取得した `ID -> model_number` のマップを返します．

戻り値:
`map<id_t, uint16_t>&`（実体は `map<uint8_t, uint16_t>&`）．直近 `Ping/Ping_broadcast` の `ID -> model_number` マップへの参照を返します．
```cpp
map<uint8_t, uint16_t>& id_model = dxl.ping_id_model_map_last_read();
```

<a id="api-dead-time-retry-ping"></a>
### `dead_time_retry_ping()`
`tryPing()` の1回分の試行で消費する目安待機時間（`num_try * latency_timer`）を返します．

戻り値:
`double`．待機時間の目安値をミリ秒単位で返します．
```cpp
double dead_time = dxl.dead_time_retry_ping();
```

<a id="api-wait-time-ping-broadcast"></a>
### `wait_time_ping_broadcast()`
`Ping_broadcast()` が内部で使用する受信待機時間を返します．

戻り値:
`double`．受信待機時間（ミリ秒）を返します．
```cpp
double wait_time = dxl.wait_time_ping_broadcast();
```
