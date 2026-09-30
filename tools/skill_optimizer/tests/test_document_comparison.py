"""模擬providerによる文書比較の回帰試験。実AI・実Git呼出しなし。"""

import argparse
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tools.skill_optimizer import compare_documents


class fake_document_provider:
    """既知の出力と利用量を返す、課金を伴わない模擬provider。"""

    def __init__(self, *, fail_at=None, unknown_usage_at=None, interrupt_at=None, change_model_at=None):
        self.calls = []
        self.fail_at = fail_at
        self.unknown_usage_at = unknown_usage_at
        self.interrupt_at = interrupt_at
        self.change_model_at = change_model_at

    def __call__(self, prompt, output_dir, *, model=None, timeout_sec=120):
        self.calls.append({"prompt": prompt, "output_dir": output_dir, "model": model, "timeout_sec": timeout_sec})
        num_calls = len(self.calls)
        value = {
            "text": f"# 整理済み原稿\n\n模擬応答番号: {num_calls}\n",
            "input_tokens": 100 if "旧版専用ルール" in prompt else 80,
            "cached_input_tokens": 20,
            "output_tokens": 20,
            "elapsed_sec": 0.01,
            "tool_calls": 0,
            "is_success": num_calls != self.fail_at,
            "usage_status": "complete",
            "model": "other-model" if num_calls == self.change_model_at else model,
            "error": "模擬実行失敗" if num_calls == self.fail_at else None,
        }
        if num_calls in (self.unknown_usage_at, self.interrupt_at):
            value.update(input_tokens=None, cached_input_tokens=None, output_tokens=None, usage_status="unknown")
        if num_calls == self.interrupt_at:
            value.update(is_success=False, error="KeyboardInterrupt")
        output_dir.mkdir(parents=True, exist_ok=False)
        (output_dir / "output.txt").write_text(value["text"], encoding="utf-8")
        (output_dir / "metadata.json").write_text(json.dumps(value, ensure_ascii=False), encoding="utf-8")
        if num_calls == self.interrupt_at:
            raise KeyboardInterrupt
        return value


class document_comparison_test(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory(prefix="document_comparison_test_")
        self.addCleanup(self.temp_dir.cleanup)
        self.root = Path(self.temp_dir.name)
        self.cases = []
        for idx in range(3):
            text = f"# 原稿 {idx}\n\n実装は未完了。処理時間 {idx + 1} ms の推定値。\n"
            self.cases.append({
                "id": f"case_{idx}", "source_path": f"docs/input_{idx}.md", "input": text,
                "source_sha256": compare_documents.sha256(text),
                "criteria": [{"id": f"private_criterion_{idx}", "text": f"採点専用秘密情報_{idx}"}],
            })
        self.cases_path = self.root / "cases.json"
        self.cases_path.write_text(json.dumps({"cases": self.cases}, ensure_ascii=False), encoding="utf-8")
        self.args = argparse.Namespace(
            cases=self.cases_path, baseline_ref="old-reference", output=self.root / "run", repeats=2,
            model="fake-model", timeout_sec=12, max_total_tokens=10000,
        )
        self.current_rules = {"skill": "新版専用ルール", "template": "新版テンプレート"}
        self.baseline_rules = {"skill": "旧版専用ルール", "template": "旧版テンプレート"}
        for name, relative in compare_documents.rule_paths.items():
            path = self.root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(self.current_rules[name], encoding="utf-8")
        self.git_calls = []

    def git_read(self, command, *, cwd, text):
        self.git_calls.append(command)
        self.assertEqual(cwd, self.root)
        self.assertTrue(text)
        if command == ["git", "rev-parse", "--verify", "old-reference^{commit}"]:
            return "a" * 40 + "\n"
        for name, path in compare_documents.rule_paths.items():
            if command == ["git", "show", "a" * 40 + ":" + path]:
                return self.baseline_rules[name]
        self.fail(f"想定外のGit読取り: {command}")

    def run_fixture(self, run_call):
        with patch.object(compare_documents, "__file__", str(self.root / "tools/skill_optimizer/compare_documents.py")), patch.object(
            compare_documents.subprocess, "check_output", side_effect=self.git_read
        ):
            return compare_documents.run(self.args, run_call=run_call)

    def test_trial_order_alternates_for_each_document_and_repeat(self):
        observed = [(case["id"], variant, repeat_idx) for case, variant, repeat_idx
                    in compare_documents.trial_order(self.cases, 2)]
        expected = [
            ("case_0", "baseline", 0), ("case_0", "current", 0),
            ("case_1", "current", 0), ("case_1", "baseline", 0),
            ("case_2", "baseline", 0), ("case_2", "current", 0),
            ("case_0", "current", 1), ("case_0", "baseline", 1),
            ("case_1", "baseline", 1), ("case_1", "current", 1),
            ("case_2", "current", 1), ("case_2", "baseline", 1),
        ]
        self.assertEqual(observed, expected)

    def test_worker_prompt_contains_no_scoring_criteria(self):
        prompt = compare_documents.make_prompt(self.cases[0], self.current_rules)
        payload = json.loads(prompt.split("原稿データ:\n", 1)[1])
        self.assertEqual(payload, {key: self.cases[0][key] for key in ("source_path", "input")})
        self.assertNotIn("private_criterion", prompt)
        self.assertNotIn("採点専用秘密情報", prompt)
        self.assertNotIn(self.cases[0]["source_sha256"], prompt)

    def test_twelve_mock_calls_complete_with_separate_blind_mapping(self):
        run_call = fake_document_provider()
        report = self.run_fixture(run_call)
        self.assertEqual(report["status"], "completed")
        self.assertEqual(report["quality_status"], "pending_blind_review")
        self.assertEqual(len(run_call.calls), 12)
        self.assertEqual(len(self.git_calls), 3)
        self.assertEqual(report["summaries"]["baseline"]["total_tokens"], 720)
        self.assertEqual(report["summaries"]["current"]["total_tokens"], 600)
        self.assertEqual(report["summaries"]["baseline"]["cached_input_tokens"], 120)
        for call in run_call.calls:
            self.assertEqual(call["model"], "fake-model")
            self.assertEqual(call["timeout_sec"], 12)
            self.assertNotIn("採点専用秘密情報", call["prompt"])
        blind = json.loads((self.args.output / "blind_review.json").read_text())
        mapping = json.loads((self.args.output / "blind_mapping.json").read_text())
        self.assertEqual(blind["cases"], self.cases)
        self.assertEqual(len(blind["answers"]), 12)
        self.assertEqual(set(mapping), {answer["id"] for answer in blind["answers"]})
        for answer in blind["answers"]:
            self.assertEqual(set(answer), {"id", "case_id", "text"})
            original = mapping[answer["id"]]
            self.assertEqual(answer["case_id"], original["case_id"])
            self.assertEqual(answer["text"], (self.args.output / original["call_dir"] / "output.txt").read_text())
        self.assertNotIn("baseline", json.dumps(blind, ensure_ascii=False))
        self.assertNotIn("current", json.dumps(blind, ensure_ascii=False))
        self.assertEqual((self.root / compare_documents.rule_paths["skill"]).read_text(), self.current_rules["skill"])

    def test_unknown_usage_is_null_and_stops_additional_calls(self):
        run_call = fake_document_provider(unknown_usage_at=3)
        with self.assertRaisesRegex(RuntimeError, "使用量欠測"):
            self.run_fixture(run_call)
        report = json.loads((self.args.output / "report.json").read_text())
        self.assertEqual(len(run_call.calls), 3)
        self.assertEqual(report["status"], "failed")
        self.assertIsNone(report["records"][-1]["input_tokens"])
        self.assertFalse(report["summaries"]["current"]["has_complete_usage"])
        self.assertIsNone(report["summaries"]["current"]["total_tokens"])
        self.assertEqual(report["summaries"]["baseline"]["total_tokens"], 120)
        self.assertFalse((self.args.output / "blind_review.json").exists())

    def test_mid_run_failure_preserves_report_without_more_calls(self):
        run_call = fake_document_provider(fail_at=4)
        with self.assertRaises(RuntimeError):
            self.run_fixture(run_call)
        report = json.loads((self.args.output / "report.json").read_text())
        metrics = json.loads((self.args.output / "metrics.json").read_text())
        self.assertEqual(len(run_call.calls), 4)
        self.assertEqual(len(report["records"]), 4)
        self.assertEqual(report["status"], "failed")
        self.assertFalse(report["records"][-1]["is_success"])
        self.assertEqual(metrics["is_completed"], 0)
        self.assertEqual(metrics["num_calls"], 4)
        self.assertFalse((self.args.output / "blind_review.json").exists())

    def test_interruption_keeps_provider_metadata_in_report(self):
        run_call = fake_document_provider(interrupt_at=3)
        with self.assertRaises(KeyboardInterrupt):
            self.run_fixture(run_call)
        report = json.loads((self.args.output / "report.json").read_text())
        self.assertEqual(len(run_call.calls), 3)
        self.assertEqual(len(report["records"]), 3)
        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["records"][-1]["error"], "KeyboardInterrupt")
        self.assertIsNone(report["summaries"]["current"]["total_tokens"])

    def test_model_change_is_not_a_valid_comparison(self):
        run_call = fake_document_provider(change_model_at=3)
        with self.assertRaisesRegex(RuntimeError, "モデル不一致"):
            self.run_fixture(run_call)
        report = json.loads((self.args.output / "report.json").read_text())
        self.assertEqual(len(run_call.calls), 3)
        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["records"][-1]["model"], "other-model")

    def test_empty_summary_does_not_report_zero_usage(self):
        report = compare_documents.summaries([])
        for variant in ("baseline", "current"):
            self.assertEqual(report[variant]["num_calls"], 0)
            self.assertFalse(report[variant]["has_complete_usage"])
            self.assertIsNone(report[variant]["total_tokens"])

    def test_exact_token_budget_prevents_next_call(self):
        self.args.max_total_tokens = 120
        run_call = fake_document_provider()
        with self.assertRaisesRegex(RuntimeError, "上限"):
            self.run_fixture(run_call)
        report = json.loads((self.args.output / "report.json").read_text())
        self.assertEqual(len(run_call.calls), 1)
        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["summaries"]["baseline"]["total_tokens"], 120)
        self.assertIsNone(report["summaries"]["current"]["total_tokens"])
        self.assertFalse((self.args.output / "calls/0002").exists())


if __name__ == "__main__":
    unittest.main()
