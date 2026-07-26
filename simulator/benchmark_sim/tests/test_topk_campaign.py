from __future__ import annotations

import unittest

from benchmark_sim.run_topk_campaign import (
    ALGORITHMS,
    TOP_K_RATES,
    TRIAL_COUNT,
    build_conditions,
    shard_size,
)


class TopKCampaignTests(unittest.TestCase):
    def test_campaign_has_six_by_six_conditions(self) -> None:
        conditions = build_conditions()

        self.assertEqual(len(conditions), len(ALGORITHMS) * len(TOP_K_RATES))
        self.assertEqual(len({condition.condition_id for condition in conditions}), 36)
        self.assertEqual(
            sorted({condition.top_k_max_cells for condition in conditions}),
            [18, 36, 90, 181, 271, 361],
        )

    def test_shards_cover_all_500_trials(self) -> None:
        sizes = [shard_size(TRIAL_COUNT, 36, index) for index in range(36)]

        self.assertEqual(sum(sizes), TRIAL_COUNT)
        self.assertEqual(set(sizes), {13, 14})


if __name__ == "__main__":
    unittest.main()
