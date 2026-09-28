import unittest

from .nav_score import Evaluator


class EvaluatorRegressionTests(unittest.TestCase):
    def test_namespace_prefixed_ground_truth_frame_matches_odom(self):
        evaluator = Evaluator(namespace="red_standard_robot1", goal=(1.0, 0.0), goal_frame="odom")
        evaluator.ground_truth_odom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "red_standard_robot1/odom")
        evaluator.odom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "odom")
        result = evaluator.finish()
        self.assertEqual(result["ground_truth_frame_mismatch_count"], 0)
        self.assertEqual(result["ground_truth_frame"], "odom")

    def test_repeated_translated_local_path_is_not_a_replan(self):
        evaluator = Evaluator(namespace="red_standard_robot1")
        evaluator.local_path([(0.0, 0.0), (1.0, 0.0), (2.0, 0.2)], "odom")
        evaluator.local_path([(0.2, 0.0), (1.2, 0.0), (2.2, 0.2)], "odom")
        self.assertEqual(evaluator.local_replans, 0)

    def test_rotating_robot_does_not_count_scan_change_as_stationary_jitter(self):
        evaluator = Evaluator()
        evaluator.odom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6, "odom")
        evaluator.scan([1.0, 1.0, 1.0], 0.1, 10.0)
        evaluator.scan([2.0, 1.0, 1.0], 0.1, 10.0)
        self.assertEqual(evaluator.stationary_scan_jitter, [])


if __name__ == "__main__":
    unittest.main()
