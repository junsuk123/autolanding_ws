import json
import os
import tempfile
import unittest

from src.pipeline import run_pipeline


class PipelineSmokeTest(unittest.TestCase):
    def test_pipeline_runs_and_returns_trajectory(self):
        root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        semantic_input = os.path.join(root, "data/samples/semantic_input_example.json")
        config = os.path.join(root, "ai/configs/policy_config.yaml")

        with tempfile.TemporaryDirectory() as _:
            summary = run_pipeline(root, semantic_input, config)

        self.assertTrue(os.path.exists(summary["out_json"]))
        self.assertTrue(os.path.exists(summary["out_csv"]))
        self.assertGreater(summary["trajectory_points"], 0)

        with open(summary["out_json"], "r", encoding="utf-8") as f:
            payload = json.load(f)
        self.assertIn("semantic_features", payload)
        self.assertIn("trajectory", payload)


if __name__ == "__main__":
    unittest.main()
