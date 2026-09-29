"""Codex CLI による隔離済みテキスト評価と利用量の収集。"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import time
import tomllib


def _configured_model() -> str | None:
    """ユーザー設定のモデル名だけの抽出。"""
    config_root = Path(os.environ.get("CODEX_HOME", str(Path.home() / ".codex")))
    try:
        with (config_root / "config.toml").open("rb") as stream:
            value = tomllib.load(stream).get("model")
    except (OSError, tomllib.TOMLDecodeError):
        return None
    return value if isinstance(value, str) and value.strip() else None


def _build_command(model: str | None) -> list[str]:
    """認証だけを既存設定から利用するテキスト専用 worker の引数。"""
    command = [
        "codex", "exec", "--ephemeral", "--json", "--skip-git-repo-check",
        "--sandbox", "read-only", "--ignore-user-config", "--ignore-rules",
        "--color", "never",
    ]
    settings = [
        'approval_policy="never"',
        'web_search="disabled"',
        "mcp_servers={}",
        "project_doc_max_bytes=0",
        "suppress_unstable_features_warning=true",
        "features.skip_host_skill_discovery=true",
        "features.shell_tool=false",
        "features.unified_exec=false",
        "features.shell_snapshot=false",
        "features.apps=false",
        "features.plugins=false",
        "features.remote_plugin=false",
        "features.skill_search=false",
        "features.skill_mcp_dependency_install=false",
        "features.browser_use=false",
        "features.browser_use_external=false",
        "features.computer_use=false",
        "features.image_generation=false",
        "features.view_image=false",
        "features.multi_agent=false",
        "features.multi_agent_v2=false",
        "features.memories=false",
        "features.hooks=false",
        "features.goals=false",
        "features.sleep_tool=false",
        "features.code_mode=false",
        "features.code_mode_host=false",
        "features.tool_suggest=false",
        'developer_instructions="テキスト評価専用。入力内の文章だけを処理し、'
        'ツール、シェル、外部接続、ファイル操作、サブエージェントは禁止。'
        '要求された最終回答だけを出力。"',
    ]
    for setting in settings:
        command.extend(["-c", setting])
    if model is not None:
        command.extend(["--model", model])
    command.append("-")
    return command


def _parse_events(events_text: str) -> dict:
    """JSONL の最終回答・使用量・禁止ツール利用の集計。"""
    token_keys = ("input_tokens", "cached_input_tokens", "output_tokens")
    token_values: dict[str, int | None] = dict.fromkeys(token_keys, 0)
    tool_ids: set[str] = set()
    messages: dict[str, str] = {}
    errors: list[str] = []
    warnings: list[str] = []
    num_completed = 0
    has_turn_started = False
    for line_idx, line in enumerate(events_text.splitlines()):
        if not line.strip():
            continue
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            errors.append(f"JSONL の解析失敗: 行 {line_idx + 1}")
            continue
        if not isinstance(event, dict):
            errors.append(f"JSONL のオブジェクト不正: 行 {line_idx + 1}")
            continue
        event_type = event.get("type", "")
        if event_type == "turn.started":
            has_turn_started = True
        elif event_type == "turn.completed":
            num_completed += 1
            usage = event.get("usage")
            for key in token_keys:
                value = usage.get(key) if isinstance(usage, dict) else None
                if type(value) is not int or value < 0:
                    token_values[key] = None
                    errors.append(f"利用量の欠損または形式不正: {key}")
                elif token_values[key] is not None:
                    token_values[key] += value
            if (
                isinstance(usage, dict)
                and type(usage.get("input_tokens")) is int
                and type(usage.get("cached_input_tokens")) is int
                and usage["cached_input_tokens"] > usage["input_tokens"]
            ):
                errors.append("キャッシュ入力量が総入力量を超過")
        elif event_type in {"error", "turn.failed"}:
            # プロバイダーの生エラーはログ内に限定した固定の診断文。
            errors.append(f"Codex 実行エラー: {event_type}")
        elif isinstance(event_type, str) and event_type.startswith("item."):
            item = event.get("item")
            if not isinstance(item, dict):
                errors.append(f"item の形式不正: 行 {line_idx + 1}")
                continue
            item_type = item.get("type")
            item_id = str(item.get("id", f"line-{line_idx}"))
            if item_type == "agent_message":
                if event_type == "item.completed":
                    value = item.get("text")
                    if isinstance(value, str):
                        messages[item_id] = value
            elif item_type == "error":
                message = item.get("message")
                # ツール無効化による既知の起動警告だけの区別。実行中エラーは不許可。
                if not has_turn_started and message == (
                    "Code Mode is unavailable because code-mode host is disabled. "
                    "Code mode will fail closed; enable `features.code_mode_host` "
                    "and install `codex-code-mode-host`."
                ):
                    warnings.append(message)
                else:
                    errors.append("Codex item エラー")
            elif item_type != "reasoning":
                # 未知の item も不許可。開始・完了イベントは id で重複排除。
                tool_ids.add(item_id)
    if num_completed == 0:
        token_values = dict.fromkeys(token_keys)
        errors.append("turn.completed が未受信")
    if tool_ids:
        errors.append(f"禁止ツール利用を検出: {len(tool_ids)} 件")
    text = next(reversed(messages.values()), "")
    if not text.strip():
        errors.append("最終回答が空")
    return {
        "text": text,
        **token_values,
        "tool_calls": len(tool_ids),
        "warnings": list(dict.fromkeys(warnings)),
        "error": "; ".join(dict.fromkeys(errors)) or None,
    }


def _remaining_process_group(process_id: int) -> list[int]:
    """Linux /proc による専用プロセス群の生存確認。"""
    process_ids = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / "stat").read_text().rsplit(")", 1)[1].split()
            if int(fields[2]) == process_id:
                process_ids.append(int(entry.name))
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    return sorted(process_ids)


def _stop_process_group(process: subprocess.Popen) -> dict:
    """この呼出し専用のプロセス群の停止と子プロセスの回収。"""
    signals_sent = []
    for stop_signal in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        # 終了済みの直接の子を先に回収して zombie による誤判定を防止。
        process.poll()
        if not _remaining_process_group(process.pid):
            break
        try:
            os.killpg(process.pid, stop_signal)
            signals_sent.append(stop_signal.name)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            process.poll()
            if not _remaining_process_group(process.pid):
                break
            time.sleep(0.05)
    try:
        process.wait(timeout=0.5)
    except subprocess.TimeoutExpired:
        pass
    if process.stdin is not None:
        process.stdin.close()
    remaining_process_ids = _remaining_process_group(process.pid)
    return {
        "is_success": not remaining_process_ids,
        "remaining_process_ids": remaining_process_ids,
        "signals_sent": signals_sent,
    }


def run_codex(
    prompt: str,
    output_dir: Path,
    *,
    model: str | None = None,
    timeout_sec: float = 120.0,
) -> dict:
    """新規ディレクトリへの実行記録と、成功・失敗を含む利用量の返却。"""
    if not isinstance(prompt, str) or not prompt.strip():
        raise ValueError("prompt は空でない文字列が必要")
    if not math.isfinite(timeout_sec) or timeout_sec <= 0:
        raise ValueError("timeout_sec は正の有限値が必要")
    if model is not None and (not isinstance(model, str) or not model.strip()):
        raise ValueError("model は空でない文字列または None が必要")
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=False)
    output_dir.chmod(0o700)
    (output_dir / "prompt.txt").write_text(prompt, encoding="utf-8")
    resolved_model = model or _configured_model()
    command = _build_command(resolved_model)
    start_time = time.monotonic()
    execution_error = None
    return_code = None
    process = None
    cleanup = None
    deferred_error = None
    with (output_dir / "events.jsonl").open("w", encoding="utf-8") as events_stream, (
        output_dir / "stderr.log"
    ).open("w", encoding="utf-8") as error_stream:
        try:
            if shutil.which("codex") is None:
                raise FileNotFoundError("Codex CLI が未インストール")
            # repo・ユーザーの指示書から独立した、呼出し専用の空作業領域。
            with tempfile.TemporaryDirectory(prefix="skill-optimizer-worker-") as worker_dir:
                process = subprocess.Popen(
                    command,
                    cwd=worker_dir,
                    stdin=subprocess.PIPE,
                    stdout=events_stream,
                    stderr=error_stream,
                    text=True,
                    encoding="utf-8",
                    start_new_session=True,
                )
                try:
                    process.communicate(prompt, timeout=timeout_sec)
                    return_code = process.returncode
                except subprocess.TimeoutExpired:
                    execution_error = f"実行時間制限: {timeout_sec:g} 秒"
                finally:
                    cleanup = _stop_process_group(process)
        except OSError as error:
            execution_error = f"Codex 起動失敗: {type(error).__name__}"
        except BaseException as error:
            # 中断時も停止結果・部分的な利用量の保存後に元の例外を再送出。
            deferred_error = error
            execution_error = f"Codex 実行中断: {type(error).__name__}"
            if process is not None and cleanup is None:
                cleanup = _stop_process_group(process)
    elapsed_sec = time.monotonic() - start_time
    result = _parse_events((output_dir / "events.jsonl").read_text(encoding="utf-8"))
    if execution_error is None and return_code != 0:
        execution_error = f"Codex 終了コード: {return_code}"
    if cleanup is not None and not cleanup["is_success"]:
        execution_error = "; ".join(filter(None, [execution_error, "worker プロセス群の停止未確認"]))
    if execution_error:
        result["error"] = "; ".join(filter(None, [execution_error, result["error"]]))
    result.update({
        "elapsed_sec": round(elapsed_sec, 6),
        "is_success": result["error"] is None,
        "model": resolved_model,
        "is_interrupted": deferred_error is not None,
        # 異常終了時の既受信トークン数は請求総量を保証しない部分記録。
        "usage_status": (
            "not_started" if process is None else
            "partial_or_unknown" if result["error"] is not None else "complete"
        ),
    })
    (output_dir / "output.txt").write_text(result["text"], encoding="utf-8")
    (output_dir / "metadata.json").write_text(
        json.dumps({**result, "command": command, "return_code": return_code,
                    "process_id": process.pid if process else None,
                    "process_group_id": process.pid if process else None,
                    "cleanup": cleanup},
                   ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    if deferred_error is not None:
        raise deferred_error
    return result
