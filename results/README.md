# Study Results

Use this directory to review completed experiments. The folders are organized
by mission or by the role the artifact played in the study.

| Location | Contents |
| --- | --- |
| [`bayesian_clue_search/`](bayesian_clue_search/README.md) | The primary 500-scenario Bayesian CLUE-search campaign and its allocator-memory comparison. |
| [`sensitivity_suite/`](sensitivity_suite/README.md) | The cross-mission sensitivity campaign: Bayesian scale and communication tests, plus collaborative known-target visit results. |
| [`hardware_pilot/`](hardware_pilot/README.md) | Imported legacy physical-test fixtures and their five-trial scenario manifest. |

The primary Bayesian campaign is in
[`bayesian_clue_search/primary_topk_campaign/`](bayesian_clue_search/primary_topk_campaign/README.md).
Its `scenario_manifest.json` is the shared ordered-scenario lock for the
Bayesian `topk_filter` study profile. It is not a miscellaneous manifest.

`allocator_replay/` and `sensitivity_suite/_cv100/` may exist in a local
checkout as generated replay captures or temporary shard outputs. They are
intentionally ignored and are not part of the published result set.
