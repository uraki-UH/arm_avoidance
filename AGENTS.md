# 開発時の記述規約

- このワークスペースで新規追加または変更する、ソースコード・設定ファイル・メッセージ定義・テストの説明コメントは日本語で記述する。
- 説明コメントは、意味を損なわない範囲で体言止めを基本とする。式・条件表記を除き、
  「〜する」「〜である」のような動詞終わりを避ける。
- ライブラリ名、ROSメッセージ名、識別子、コマンドなど、技術上そのまま表記する必要がある語句は英字のまま使用してよい。
- 既存の英語コメントを変更する場合も、変更範囲では日本語へ置き換える。
- 識別子・ROSパラメータ名は lower_snake_case で統一し、最小値は `min_*`、最大値は
  `max_*` とする。`minimum` と `maximum` は使わない。
- 判定用の閾値は `*_th` と命名する。`threshold` は識別子・ROSパラメータ名に使わない。
- `min`・`max`・`*_th` を含む識別子・ROSパラメータ名では、比較方向を名前で表す。
  対応するコメントに不等号や「この値以上／以下」を重ねて書かず、測定対象・単位・用途だけを記す。
- 三角関数を表す識別子・ROSパラメータ名は、`cosine` と `sine` ではなく `cos` と
  `sin` を使う。
- 識別子・ROSパラメータ名で偏差を表す場合は、`deviation` ではなく `dev` を使う。
- 真偽値の識別子・ROSパラメータ名は、意味に応じて `is_*`（状態・判定）、`has_*`（有無）、
  `can_*`（能力）、`enable_*`（機能のON/OFF）、`allow_*`（許可）から始める。曖昧な動詞だけの
  真偽値名は使わない。
- 作業中にエージェント自身が起動したROSノード・再生・ベンチマークなどの常駐プロセスは、
  作業完了前に停止する。既存プロセスの停止は、ユーザーが明示的に依頼した場合だけ行う。
- エージェントが新規プロセスを起動した場合は、最終報告に起動コマンドと停止済みであることを
  必ず記載する。

## Git worktree同期フロー

- 各worktreeの基準ブランチは、`branch.<worktreeブランチ名>.codex-base`へ明示的に保存する。
  元workspaceで現在checkoutされているブランチを、暗黙の基準として扱わない。
- 新しいタスクでファイルを変更する前に、リポジトリルートで
  `./scripts/worktree_status.sh`を実行する。ファイル本文の全走査による比較は行わない。
- 基準ブランチが未設定の場合は変更作業を開始せず、対象を確認してから
  `./scripts/worktree_update.sh --set-base <branch>`で設定する。
- `behind > 0`かつworktreeがcleanの場合は、変更作業の前に
  `./scripts/worktree_update.sh`を実行する。worktree固有コミットと基準ブランチが分岐している場合は、
  内容を確認してから`./scripts/worktree_update.sh --rebase`を明示的に実行する。
- worktreeがdirtyの場合、自動stash、自動commit、自動reset、基準ブランチの取り込みを行わない。
  既存変更の所有者と内容を確認し、退避またはcommitの方針を決めてから同期する。
- タスク開始後に基準ブランチまたは元workspaceのcheckoutブランチが変わっても、作業途中では
  自動追従しない。次のタスク開始時に同期状態を再確認する。
- 元workspaceのcheckoutブランチと基準ブランチが異なる場合、`is_root_mismatch=yes`を警告として扱う。
  基準ブランチの変更はユーザーの指示または明示的な作業対象変更がある場合だけ行う。
- worktreeの成果はcommit単位で基準ブランチへmergeまたはcherry-pickする。通常の反映手段として、
  元workspaceへのファイルコピーや未コミットpatch適用を使わない。
- `git reset --hard`、強制checkout、未確認のstash削除など、既存変更を失う可能性がある同期操作は禁止。

## 識別子・ROSパラメータ名の省略形辞書

| 使用する表記 | 意味 | 用途 | 識別子・ROSパラメータ名で避ける表記 |
| --- | --- | --- | --- |
| `min` | minimum | 下限 | `minimum`、下限しきい値を表す `floor` |
| `max` | maximum | 上限 | `maximum` |
| `th` | threshold | 判定しきい値 | `threshold` |
| `num` | number | 個数 | `number` |
| `idx` | index | 添字 | `index` |
| `req` | requirement | 要求数・要求条件 | `requirement` |
| `dist` | distance | 距離 | `distance` |
| `iter` | iteration | 反復回数・反復添字 | `iteration`、`iterations` |
| `deg` | degree | 角度の単位 | `degree` |
| `cos` | cosine | 余弦値 | `cosine` |
| `sin` | sine | 正弦値 | `sine` |
| `dev` | deviation | 基準からの偏差 | `deviation` |

- 新しい省略形は、この辞書へ意味と用途を追加してから使用する。
