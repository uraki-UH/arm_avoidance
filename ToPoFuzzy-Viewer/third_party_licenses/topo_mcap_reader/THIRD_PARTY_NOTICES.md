# topo_mcap_reader.js 第三者ライセンス

`topo_mcap_reader.js`へ組み込まれる第三者パッケージとライセンス本文の対応表。
配布物には本ディレクトリ全体を同梱すること。

| パッケージ | ライセンス本文 |
| --- | --- |
| `@foxglove/cdr`, `@foxglove/crc`, `@foxglove/rosmsg`, `@foxglove/rosmsg2-serialization`, `@foxglove/wasm-bz2`, `@foxglove/wasm-lz4`, `@foxglove/wasm-zstd` | `Foxglove-MIT.txt` |
| `@mcap/browser`, `@mcap/core`, `@mcap/support` | `Foxglove-MIT.txt` |
| `@protobufjs/aspromise`, `@protobufjs/base64`, `@protobufjs/codegen`, `@protobufjs/eventemitter`, `@protobufjs/fetch`, `@protobufjs/float`, `@protobufjs/path`, `@protobufjs/pool`, `@protobufjs/utf8`, `protobufjs` | `protobufjs-BSD-3-Clause.txt` |
| `heap-js` | `heap-js-BSD-3-Clause.txt` |
| `long` | `long-Apache-2.0.txt` |
| `tslib` | `tslib-0BSD.txt` |

対象パッケージはesbuildのmetafileから抽出し、バージョンとSPDX識別子は
`frontend/package-lock.json`を基準とする。
