"""模擬 provider による制御系テスト。実 API の品質改善実証とは別の検証。"""

from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import signal
import tempfile
import unittest
from unittest.mock import patch

from tools.skill_optimizer.optimizer import (
    compare,
    estimated_cost,
    grade,
    load_experiment,
    optimize,
    read_json,
    validate_candidate,
)


frontmatter = "---\nname: fixture-skill\ndescription: テスト専用の抽出手順\n---\n"
baseline_skill = frontmatter + "BASELINE: 入力内容を順に検査し、結果だけを返却。\n"
candidate_skill = frontmatter + "CANDIDATE: 必須値を抽出し、結果だけを返却。\n"


class fake_provider:
    """入力ごとの既知応答と既知 token 数による模擬モデル。"""

    def __init__(self, cases, *, proposals=None, candidate_errors=None, unknown_usage_at=None,
                 fail_at=None, tool_calls_at=None, candidate_input_tokens=60):
        self.expected = {case["input"]: case["expected"] for case in cases}
        self.proposals = proposals or [candidate_skill]
        self.candidate_errors = candidate_errors or set()
        self.unknown_usage_at = unknown_usage_at
        self.fail_at = fail_at
        self.tool_calls_at = tool_calls_at
        self.candidate_input_tokens = candidate_input_tokens
        self.calls = []
        self.num_proposals = 0

    def __call__(self, prompt, output_dir, *, model=None, timeout_sec=60):
        payload = json.loads(prompt.rsplit("\n", 1)[1])
        is_teacher = "training_traces" in payload
        if is_teacher:
            text = self.proposals[min(self.num_proposals, len(self.proposals) - 1)]
            self.num_proposals += 1
            input_tokens, output_tokens = 150, 30
        else:
            is_candidate = payload["skill"] != baseline_skill
            value = self.expected[payload["task_input"]]
            if is_candidate and payload["task_input"] in self.candidate_errors:
                value = {"value": "誤答"}
            text = json.dumps(value, ensure_ascii=False)
            input_tokens = self.candidate_input_tokens if is_candidate else 120
            output_tokens = 10 if is_candidate else 20
        num_calls = len(self.calls) + 1
        result = {
            "text": text,
            "input_tokens": input_tokens,
            "cached_input_tokens": 0,
            "output_tokens": output_tokens,
            "tool_calls": int(num_calls == self.tool_calls_at),
            "elapsed_sec": 0.01,
            "is_success": num_calls != self.fail_at,
            "error": "模擬実行失敗" if num_calls == self.fail_at else None,
            "model": model,
        }
        if num_calls == self.unknown_usage_at:
            result["input_tokens"] = None
        self.calls.append({"payload": payload, "directory": output_dir, "model": model,
                           "timeout_sec": timeout_sec, "is_teacher": is_teacher})
        return result


class strict_grading_test(unittest.TestCase):
    def test_json_types_and_structure(self):
        expected = {"count": 1, "items": [True, None]}
        self.assertEqual(grade('{"items":[true,null],"count":1}', expected, "exact_json")["score"], 1)
        invalid_outputs = (
            '{"count":true,"items":[true,null]}',
            '{"count":1.0,"items":[true,null]}',
            '{"count":1,"items":[1,null]}',
            '{"count":1,"items":[true,null],"extra":0}',
            '{"count":1}',
            '{"count":1,"items":[null,true]}',
            '```json\n{"count":1,"items":[true,null]}\n```',
        )
        for text in invalid_outputs:
            with self.subTest(text=text):
                self.assertEqual(grade(text, expected, "exact_json")["score"], 0)

    def test_duplicate_and_nonfinite_json_values(self):
        for text in ('{"a":1,"a":1}', '{"x":{"a":1,"a":2}}',
                     '{"a":NaN}', '{"a":Infinity}', '{"a":-Infinity}', '{"a":1e400}'):
            with self.subTest(text=text):
                with self.assertRaises(ValueError):
                    read_json(text)

    def test_exact_text_only_trims_outer_whitespace(self):
        self.assertEqual(grade("  正解\n", "正解", "exact_text")["score"], 1)
        self.assertEqual(grade("正 解", "正解", "exact_text")["score"], 0)

    def test_candidate_body_and_frontmatter_constraints(self):
        self.assertEqual(validate_candidate(candidate_skill, baseline_skill), candidate_skill)
        for candidate in ("指示のみ", frontmatter, candidate_skill.replace("fixture-skill", "other"),
                          frontmatter + "長" * 24000):
            with self.subTest(candidate=candidate[:80]):
                with self.assertRaises(ValueError):
                    validate_candidate(candidate, baseline_skill)

    def test_cached_input_not_charged_twice(self):
        result = {"input_tokens": 1000, "cached_input_tokens": 600, "output_tokens": 20}
        prices = {"input_per_million": 10, "cached_input_per_million": 1, "output_per_million": 20}
        self.assertAlmostEqual(estimated_cost(result, prices), 0.005)
        self.assertIsNone(estimated_cost(result, None))
        self.assertIsNone(estimated_cost({**result, "cached_input_tokens": None}, prices))

    def test_case_regression_rejected_at_unchanged_overall_quality(self):
        def record(case_id, score, tokens):
            return {"case_id": case_id, "repeat": 0, "grade": {"score": score},
                    "input_tokens": tokens, "cached_input_tokens": 0, "output_tokens": 1,
                    "tool_calls": 0, "elapsed_sec": 0.01, "is_success": True, "estimated_cost": None}

        before = [record("first", 1, 100), record("second", 0, 100)]
        after = [record("first", 0, 10), record("second", 1, 10)]
        decision = compare(before, after, {"min_quality": 0.5, "min_token_improvement_ratio": 0.1})
        self.assertFalse(decision["is_accepted"])
        self.assertIn("個別ケースの品質低下", decision["reasons"])

    def test_exact_token_saving_boundary_is_accepted(self):
        before = {"case_id": "case", "repeat": 0, "grade": {"score": 1},
                  "input_tokens": 90, "cached_input_tokens": 0, "output_tokens": 10,
                  "tool_calls": 0, "elapsed_sec": 0.01, "is_success": True, "estimated_cost": None}
        config = {"min_quality": 1, "min_token_improvement_ratio": 0.1}
        after = {**before, "input_tokens": 80}
        decision = compare([before], [after], config)
        self.assertTrue(decision["is_accepted"])
        self.assertAlmostEqual(decision["token_saving_ratio"], 0.1)
        decision = compare([before], [{**after, "input_tokens": 81}], config)
        self.assertFalse(decision["is_accepted"])


class optimizer_integration_test(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory(prefix="skill_optimizer_test_")
        self.addCleanup(self.temp_dir.cleanup)
        self.root = Path(self.temp_dir.name)
        self.cases = [
            {"id": "train_a", "split": "train", "input": "学習専用入力_A", "expected": {"value": "学習正解_A"}},
            {"id": "validation_a", "split": "validation", "input": "選択専用入力_B", "expected": {"value": "選択正解_B"}},
            {"id": "test_a", "split": "test", "input": "最終専用入力_C", "expected": {"value": "最終正解_C"}},
        ]
        self.config = {
            "skill": "SKILL.md", "policy": "policy.md", "dataset": "dataset.json", "grader": "exact_json",
            "worker_model": "fake-worker", "teacher_model": "fake-teacher", "max_rounds": 1,
            "repeats": 2, "max_calls": 30, "max_total_tokens": 100000, "timeout_sec": 60,
            "max_total_sec": 300, "min_quality": 1.0, "min_token_improvement_ratio": 0.1,
        }
        self.manifest = self.root / "experiment.json"
        (self.root / "SKILL.md").write_text(baseline_skill, encoding="utf-8")
        (self.root / "policy.md").write_text("指定値だけを JSON へ変換。", encoding="utf-8")
        self.write_fixture()

    def write_fixture(self):
        self.manifest.write_text(json.dumps(self.config, ensure_ascii=False), encoding="utf-8")
        (self.root / "dataset.json").write_text(json.dumps({"cases": self.cases}, ensure_ascii=False), encoding="utf-8")

    def run_fixture(self, provider=None, output_name="run"):
        self.write_fixture()
        provider = provider or fake_provider(self.cases)
        with redirect_stdout(io.StringIO()):
            report = optimize(self.manifest, self.root / output_name, provider=provider)
        return report, provider

    def test_full_adoption_and_original_preservation(self):
        originals = {name: (self.root / name).read_bytes() for name in ("SKILL.md", "policy.md", "dataset.json", "experiment.json")}
        report, provider = self.run_fixture()
        self.assertEqual(report["status"], "completed")
        self.assertTrue(report["is_accepted"])
        self.assertEqual(report["selected"], "candidate_1")
        self.assertEqual(report["final_comparison"]["token_saving_ratio"], 0.5)
        self.assertEqual(len(provider.calls), 11)
        self.assertEqual((self.root / "run/selected.SKILL.md").read_text(), candidate_skill)
        self.assertEqual(json.loads((self.root / "run/report.json").read_text())["selected"], "candidate_1")
        self.assertTrue((self.root / "run/candidate_1.diff").is_file())
        for name, content in originals.items():
            self.assertEqual((self.root / name).read_bytes(), content)

    def test_teacher_sees_only_training_and_final_runs_after_search(self):
        self.config["max_rounds"] = 2
        report, provider = self.run_fixture()
        teacher_positions = []
        test_positions = []
        for idx, call in enumerate(provider.calls):
            if call["is_teacher"]:
                teacher_positions.append(idx)
                self.assertEqual(call["model"], "fake-teacher")
                text = json.dumps(call["payload"], ensure_ascii=False)
                for case in self.cases[1:]:
                    self.assertNotIn(case["input"], text)
                    self.assertNotIn(case["expected"]["value"], text)
                self.assertEqual(len(call["payload"]["training_traces"]), 1)
            else:
                self.assertEqual(call["model"], "fake-worker")
                if call["payload"]["task_input"] == self.cases[2]["input"]:
                    test_positions.append(idx)
                self.assertNotIn("expected", call["payload"])
        self.assertEqual(len(teacher_positions), 2)
        self.assertEqual(len(test_positions), 4)
        self.assertGreater(min(test_positions), max(teacher_positions))
        self.assertTrue(report["is_accepted"])

    def test_validation_acceptance_followed_by_heldout_regression(self):
        provider = fake_provider(self.cases, candidate_errors={self.cases[2]["input"]})
        report, _ = self.run_fixture(provider)
        self.assertTrue(report["rounds"][0]["is_accepted"])
        self.assertEqual(report["validation_selected"], "candidate_1")
        self.assertFalse(report["final_comparison"]["is_accepted"])
        self.assertEqual(report["selected"], "baseline")
        self.assertFalse(report["is_accepted"])
        self.assertEqual((self.root / "run/selected.SKILL.md").read_text(), baseline_skill)

    def test_rejected_proposal_history_contains_only_training_feedback(self):
        self.config["max_rounds"] = 2
        provider = fake_provider(self.cases, candidate_errors={case["input"] for case in self.cases[:2]})
        report, provider = self.run_fixture(provider)
        self.assertFalse(report["rounds"][0]["is_accepted"])
        self.assertEqual(report["rounds"][0]["candidate_name"], "candidate_1")
        teacher_calls = [call["payload"] for call in provider.calls if call["is_teacher"]]
        self.assertEqual(len(teacher_calls), 2)
        self.assertEqual(teacher_calls[0]["previous_proposals_training_only"], [])
        second = teacher_calls[1]
        self.assertEqual(second["current_skill"], baseline_skill)
        self.assertEqual(second["training_traces"][0]["grade"]["score"], 1)
        history = second["previous_proposals_training_only"]
        self.assertEqual(len(history), 1)
        self.assertEqual(history[0]["skill"], candidate_skill)
        self.assertEqual(history[0]["training_summary"]["quality"], 0)
        self.assertEqual(history[0]["training_summary"]["total_tokens"], 70)
        feedback = history[0]["training_feedback"]
        self.assertEqual(len(feedback), 1)
        self.assertEqual(feedback[0]["input"], self.cases[0]["input"])
        self.assertEqual(feedback[0]["expected"], self.cases[0]["expected"])
        self.assertEqual(feedback[0]["grade"]["score"], 0)
        for payload in teacher_calls:
            text = json.dumps(payload, ensure_ascii=False)
            for case in self.cases[1:]:
                self.assertNotIn(case["id"], text)
                self.assertNotIn(case["input"], text)
                self.assertNotIn(case["expected"]["value"], text)

    def test_cheaper_wrong_candidate_rejected(self):
        provider = fake_provider(self.cases, candidate_errors={self.cases[1]["input"]}, candidate_input_tokens=1)
        report, _ = self.run_fixture(provider)
        self.assertFalse(report["rounds"][0]["is_accepted"])
        self.assertEqual(report["selected"], "baseline")

    def test_correct_but_insufficient_saving_rejected(self):
        provider = fake_provider(self.cases, candidate_input_tokens=125)
        report, _ = self.run_fixture(provider)
        self.assertFalse(report["is_accepted"])
        self.assertIn("総トークン削減が採用下限未達", report["rounds"][0]["reasons"])

    def test_unknown_usage_provider_failure_and_tools_fail_closed(self):
        for key in ("unknown_usage_at", "fail_at", "tool_calls_at"):
            with self.subTest(key=key):
                provider = fake_provider(self.cases, **{key: 4})
                report, provider = self.run_fixture(provider, output_name=key)
                self.assertEqual(report["status"], "failed")
                self.assertEqual(len(provider.calls), 4)
                self.assertFalse(report["is_accepted"])
                self.assertEqual(report["selected"], "baseline")
                self.assertEqual((self.root / key / "selected.SKILL.md").read_text(), baseline_skill)

    def test_call_budget_stops_and_retains_report(self):
        self.config["max_calls"] = 3
        report, provider = self.run_fixture()
        self.assertEqual(len(provider.calls), 3)
        self.assertEqual(report["status"], "failed")
        self.assertIn("max_calls", report["error"])
        self.assertEqual(report["selected"], "baseline")
        self.assertTrue((self.root / "run/report.md").is_file())

    def test_token_budget_stops_after_crossing_call(self):
        self.config["max_total_tokens"] = 200
        report, provider = self.run_fixture()
        self.assertEqual(len(provider.calls), 2)
        self.assertEqual(report["total_tokens"], 320)
        self.assertIn("max_total_tokens", report["error"])
        self.assertFalse(report["is_accepted"])

    def test_elapsed_budget_prevents_next_call(self):
        self.config["max_total_sec"] = 1
        elapsed = [0.0]
        provider = fake_provider(self.cases)

        def advance_clock(prompt, output_dir, **kwargs):
            result = provider(prompt, output_dir, **kwargs)
            elapsed[0] += 2.0
            return result

        with patch("tools.skill_optimizer.optimizer.time.monotonic", side_effect=lambda: elapsed[0]):
            report, _ = self.run_fixture(advance_clock)
        self.assertEqual(len(provider.calls), 1)
        self.assertEqual(provider.calls[0]["timeout_sec"], 1)
        self.assertIn("max_total_sec", report["error"])
        self.assertEqual(report["selected"], "baseline")

    def test_cancellation_retains_baseline_and_restores_handlers(self):
        handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}

        def interrupted_provider(*args, **kwargs):
            raise KeyboardInterrupt

        report, _ = self.run_fixture(interrupted_provider)
        self.assertEqual(report["status"], "cancelled")
        self.assertFalse(report["is_accepted"])
        self.assertFalse(report["has_complete_usage"])
        self.assertEqual(len(report["calls"]), 1)
        self.assertIsNone(report["calls"][0]["input_tokens"])
        self.assertIsNone(report["calls"][0]["output_tokens"])
        self.assertEqual(report["calls"][0]["error"], "KeyboardInterrupt")
        self.assertIn("使用量欠測", (self.root / "run/report.md").read_text())
        self.assertEqual((self.root / "run/selected.SKILL.md").read_text(), baseline_skill)
        for sig, handler in handlers.items():
            self.assertEqual(signal.getsignal(sig), handler)

    def test_model_change_stops_comparison(self):
        provider = fake_provider(self.cases)

        def changing_provider(prompt, output_dir, **kwargs):
            result = provider(prompt, output_dir, **kwargs)
            if len(provider.calls) == 3:
                result["model"] = "different-worker"
            return result

        report, _ = self.run_fixture(changing_provider)
        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["resolved_models"]["worker"], "fake-worker")
        self.assertEqual(report["calls"][-1]["model"], "different-worker")
        self.assertEqual(len(provider.calls), 3)
        self.assertIn("試験中にモデルが変更", report["error"])
        self.assertFalse(report["is_accepted"])
        self.assertEqual(report["selected"], "baseline")

    def test_usage_gap_does_not_claim_prices_are_missing(self):
        self.config["prices"] = {
            role: {"input_per_million": 2, "cached_input_per_million": 0.2, "output_per_million": 6}
            for role in ("worker", "teacher")
        }
        report, _ = self.run_fixture(fake_provider(self.cases, unknown_usage_at=4))
        self.assertFalse(report["has_complete_usage"])
        self.assertIsNone(report["estimated_cost"])
        markdown = (self.root / "run/report.md").read_text()
        self.assertIn("金額見積り: 使用量欠測", markdown)
        self.assertNotIn("単価未設定", markdown)

    def test_paired_evaluation_alternates_execution_order(self):
        _, provider = self.run_fixture()
        for case in self.cases[1:]:
            observed = [call["payload"]["skill"] != baseline_skill for call in provider.calls
                        if not call["is_teacher"] and call["payload"]["task_input"] == case["input"]]
            self.assertEqual(observed, [False, True, True, False])

    def test_report_accounts_for_teacher_and_worker_prices(self):
        self.config["prices"] = {
            "worker": {"input_per_million": 2, "cached_input_per_million": 0.2, "output_per_million": 6},
            "teacher": {"input_per_million": 5, "cached_input_per_million": 0.5, "output_per_million": 15},
        }
        report, _ = self.run_fixture()
        self.assertAlmostEqual(report["estimated_cost"], 0.0039)
        self.assertAlmostEqual(report["summaries"]["test_baseline"]["estimated_cost"], 0.00072)
        self.assertIsNone(report["break_even_runs"])

    def test_output_directory_cannot_be_reused(self):
        self.run_fixture()
        previous = (self.root / "run/report.json").read_bytes()
        with self.assertRaises(FileExistsError):
            self.run_fixture()
        self.assertEqual((self.root / "run/report.json").read_bytes(), previous)

    def test_frontmatter_mutation_is_rejected_without_candidate_execution(self):
        provider = fake_provider(self.cases, proposals=[candidate_skill.replace("fixture-skill", "changed")])
        report, provider = self.run_fixture(provider)
        self.assertEqual(report["status"], "completed")
        self.assertFalse(report["rounds"][0]["is_accepted"])
        self.assertFalse(any(not call["is_teacher"] and call["payload"]["skill"] != baseline_skill for call in provider.calls))

    def test_manifest_rejects_boolean_limits_and_split_leakage(self):
        for key in ("max_calls", "max_total_tokens", "repeats", "timeout_sec", "min_quality"):
            with self.subTest(key=key):
                original = self.config[key]
                self.config[key] = True
                self.write_fixture()
                with self.assertRaises(ValueError):
                    load_experiment(self.manifest)
                self.config[key] = original
        self.cases[2]["input"] = self.cases[0]["input"]
        self.write_fixture()
        with self.assertRaisesRegex(ValueError, "split間漏洩"):
            load_experiment(self.manifest)


if __name__ == "__main__":
    unittest.main()
