"""CLI 実行記録・未知の利用量・異常終了の回帰試験。"""

import json
from pathlib import Path
import signal
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch

from tools.skill_optimizer import provider


def event_log(*extra_events, usage=None):
    """最終回答と使用量を含む最小イベント列。"""
    if usage is None:
        usage = {"input_tokens": 100, "cached_input_tokens": 20, "output_tokens": 8}
    return "\n".join(json.dumps(event) for event in [
        {"type": "item.completed", "item": {
            "id": "answer", "type": "agent_message", "text": '{"ok":true}',
        }},
        {"type": "turn.completed", "usage": usage},
        *extra_events,
    ]) + "\n"


class provider_test(unittest.TestCase):
    """課金対象の実モデル呼出しを伴わない provider 試験。"""

    def test_parse_usage(self):
        result = provider._parse_events(event_log())
        self.assertEqual(result["input_tokens"], 100)
        self.assertEqual(result["cached_input_tokens"], 20)
        self.assertEqual(result["output_tokens"], 8)
        self.assertEqual(result["tool_calls"], 0)
        self.assertIsNone(result["error"])

    def test_multiple_turn_usage(self):
        result = provider._parse_events(event_log({
            "type": "turn.completed",
            "usage": {"input_tokens": 5, "cached_input_tokens": 0, "output_tokens": 3},
        }))
        self.assertEqual(result["input_tokens"], 105)
        self.assertEqual(result["cached_input_tokens"], 20)
        self.assertEqual(result["output_tokens"], 11)

    def test_missing_usage_is_unknown(self):
        result = provider._parse_events(event_log(usage={"output_tokens": 0}))
        self.assertIsNone(result["input_tokens"])
        self.assertIsNone(result["cached_input_tokens"])
        self.assertEqual(result["output_tokens"], 0)
        self.assertIsNotNone(result["error"])

    def test_invalid_usage_is_not_zero(self):
        for invalid_value in (True, -1, "10", 1.2):
            with self.subTest(invalid_value=invalid_value):
                result = provider._parse_events(event_log(usage={
                    "input_tokens": invalid_value, "cached_input_tokens": 0,
                    "output_tokens": 1,
                }))
                self.assertIsNone(result["input_tokens"])
                self.assertIsNotNone(result["error"])

    def test_cached_input_cannot_exceed_total(self):
        result = provider._parse_events(event_log(usage={
            "input_tokens": 2, "cached_input_tokens": 3, "output_tokens": 1,
        }))
        self.assertIsNotNone(result["error"])

    def test_tools_are_rejected_and_deduplicated(self):
        for item_type in ("command_execution", "mcp_tool_call", "file_change", "unknown"):
            with self.subTest(item_type=item_type):
                item = {"id": "tool", "type": item_type}
                result = provider._parse_events(event_log(
                    {"type": "item.started", "item": item},
                    {"type": "item.completed", "item": item},
                ))
                self.assertEqual(result["tool_calls"], 1)
                self.assertIsNotNone(result["error"])

    def test_malformed_or_failed_events_are_rejected(self):
        for raw in ("", "not-json", "[]", event_log({"type": "turn.failed"})):
            with self.subTest(raw=raw):
                self.assertIsNotNone(provider._parse_events(raw)["error"])

    def test_only_exact_startup_warning_is_nonfatal(self):
        message = (
            "Code Mode is unavailable because code-mode host is disabled. "
            "Code mode will fail closed; enable `features.code_mode_host` "
            "and install `codex-code-mode-host`."
        )
        warning = json.dumps({"type": "item.completed", "item": {
            "id": "warning", "type": "error", "message": message,
        }}) + "\n"
        result = provider._parse_events(warning + event_log())
        self.assertIsNone(result["error"])
        self.assertEqual(result["warnings"], [message])
        for raw in (
            warning.replace(message, "unknown warning") + event_log(),
            '{"type":"turn.started"}\n' + warning + event_log(),
        ):
            self.assertIsNotNone(provider._parse_events(raw)["error"])

    def test_command_preserves_read_only_sandbox(self):
        command = provider._build_command("test-model")
        self.assertEqual(command[command.index("--sandbox") + 1], "read-only")
        self.assertEqual(command[command.index("--model") + 1], "test-model")
        self.assertIn("--ignore-user-config", command)
        self.assertIn('web_search="disabled"', command)
        self.assertIn("features.shell_tool=false", command)
        self.assertFalse(any("bypass" in argument for argument in command))

    def test_success_logs_and_cleanup(self):
        process = Mock(pid=999999, returncode=0)
        cleanup = {"is_success": True, "remaining_process_ids": [], "signals_sent": []}

        def launch(command, **kwargs):
            kwargs["stdout"].write(event_log())
            self.assertTrue(kwargs["start_new_session"])
            self.assertEqual(list(Path(kwargs["cwd"]).iterdir()), [])
            return process

        with tempfile.TemporaryDirectory() as directory, patch.object(
            provider.subprocess, "Popen", side_effect=launch
        ), patch.object(provider.shutil, "which", return_value="/test/codex"), patch.object(
            provider, "_stop_process_group", return_value=cleanup
        ) as stop:
            output_dir = Path(directory) / "call"
            result = provider.run_codex("test", output_dir, model="test-model")
            self.assertTrue(result["is_success"])
            self.assertEqual(result["model"], "test-model")
            self.assertEqual(set(entry.name for entry in output_dir.iterdir()), {
                "prompt.txt", "events.jsonl", "stderr.log", "output.txt", "metadata.json",
            })
            metadata = json.loads((output_dir / "metadata.json").read_text())
            self.assertEqual(metadata["process_group_id"], process.pid)
            self.assertEqual(metadata["cleanup"], cleanup)
            stop.assert_called_once_with(process)
            with self.assertRaises(FileExistsError):
                provider.run_codex("test", output_dir, model="test-model")

    def test_timeout_keeps_logs_and_is_failure(self):
        process = Mock(pid=999999, returncode=None)
        process.communicate.side_effect = subprocess.TimeoutExpired("codex", 0.01)
        cleanup = {"is_success": True, "remaining_process_ids": [], "signals_sent": ["SIGINT"]}
        with tempfile.TemporaryDirectory() as directory, patch.object(
            provider.subprocess, "Popen", return_value=process
        ), patch.object(provider.shutil, "which", return_value="/test/codex"), patch.object(
            provider, "_stop_process_group", return_value=cleanup
        ) as stop:
            result = provider.run_codex("test", Path(directory) / "call", model="test", timeout_sec=0.01)
            self.assertFalse(result["is_success"])
            self.assertIn("実行時間制限", result["error"])
            self.assertIsNone(result["input_tokens"])
            stop.assert_called_once_with(process)

    def test_missing_cli_is_failure(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(
            provider.shutil, "which", return_value=None
        ):
            result = provider.run_codex("test", Path(directory) / "call", model="test")
            self.assertFalse(result["is_success"])
            self.assertIn("FileNotFoundError", result["error"])

    def test_interruption_saves_partial_usage_and_cleanup_before_reraise(self):
        for raw in (event_log(), '{"type":"turn.started"}\n'):
            with self.subTest(has_usage="turn.completed" in raw):
                process = Mock(pid=999999, returncode=None)
                process.communicate.side_effect = KeyboardInterrupt()
                cleanup = {
                    "is_success": True, "remaining_process_ids": [], "signals_sent": ["SIGINT"],
                }

                def launch(command, **kwargs):
                    kwargs["stdout"].write(raw)
                    return process

                with tempfile.TemporaryDirectory() as directory, patch.object(
                    provider.subprocess, "Popen", side_effect=launch
                ), patch.object(provider.shutil, "which", return_value="/test/codex"), patch.object(
                    provider, "_stop_process_group", return_value=cleanup
                ) as stop:
                    output_dir = Path(directory) / "call"
                    with self.assertRaises(KeyboardInterrupt):
                        provider.run_codex("test", output_dir, model="test")
                    metadata = json.loads((output_dir / "metadata.json").read_text())
                    self.assertFalse(metadata["is_success"])
                    self.assertTrue(metadata["is_interrupted"])
                    self.assertEqual(metadata["usage_status"], "partial_or_unknown")
                    self.assertEqual(metadata["process_id"], process.pid)
                    self.assertEqual(metadata["cleanup"], cleanup)
                    self.assertIn("KeyboardInterrupt", metadata["error"])
                    self.assertEqual(
                        metadata["input_tokens"], 100 if "turn.completed" in raw else None,
                    )
                    self.assertTrue((output_dir / "output.txt").exists())
                    stop.assert_called_once_with(process)

    def test_cleanup_signals_only_owned_group(self):
        process = Mock(pid=999999)
        with patch.object(provider, "_remaining_process_group", side_effect=[
            [999999], [], [], [],
        ]), patch.object(provider.os, "killpg") as kill_group:
            result = provider._stop_process_group(process)
        kill_group.assert_called_once_with(999999, signal.SIGINT)
        self.assertTrue(result["is_success"])
        self.assertEqual(result["signals_sent"], ["SIGINT"])

    def test_cleanup_escalates_only_while_group_remains(self):
        process = Mock(pid=999999)
        with patch.object(provider, "_remaining_process_group", side_effect=[
            [999999], [999999], [999999], [],
        ]), patch.object(provider.time, "monotonic", side_effect=range(6)), patch.object(
            provider.os, "killpg"
        ) as kill_group:
            result = provider._stop_process_group(process)
        self.assertEqual(kill_group.call_count, 3)
        self.assertTrue(result["is_success"])
        self.assertEqual(result["signals_sent"], ["SIGINT", "SIGTERM", "SIGKILL"])


if __name__ == "__main__":
    unittest.main()
