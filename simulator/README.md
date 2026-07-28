# Bayesian DCTA Simulator

The `benchmark_sim` package is the complete clue-informed and coverage
simulation architecture used by the Top-K filtering study.

The default `topk_filter` study profile fails fast unless a run uses the
canonical 19x19 clue-search configuration, four evenly spaced left-edge robots,
and commitment horizon 3. Use `--study-profile custom` only for a deliberate
non-study or legacy sensitivity run.

Key components:

- `benchmark_sim/core/belief.py`: per-robot Bayesian target belief
- `benchmark_sim/core/`: world, robot, planner, scheduler, and scenario loading
- `benchmark_sim/algorithms/`: allocation algorithms and registry
- `benchmark_sim/algorithms/DGA.py`: default compact-population DGA
- `benchmark_sim/algorithms/DGA_optimized.py`: compact implementation module
- `benchmark_sim/algorithms/memory_optimized.py`: shared packed-candidate and
  dense cell-table primitives used by the default ACBBA, CBAA, HIPC, and PI
  allocators
- `benchmark_sim/comms/`: message bus and degraded communication models
- `benchmark_sim/metrics/`: trial, robot, and system metric exports
- `benchmark_sim/visualization/`: optional pygame viewer
- `benchmark_sim/tests/horizon_topk/`: Top-K and horizon study drivers
- `scenarios/`: bundled scenario inputs

Run commands from this directory. The candidate prefilter is controlled by
`--max-candidate-cells <n|all>` or `--top-k-rate <rate>`. The rate form
preserves the requested rate in every output CSV and converts it to an integer
candidate limit with round-half-up. Write study output to
`../results/<run-name>` to keep generated results separate from simulator
source.

Scenario loading is strict: IDs and coordinates must be integers; IDs and
target/clue cells must be unique where required; coordinates must be in bounds;
and a target may not overlap a robot start. A clue on a start is valid and is
sensed at logical time zero after every robot and allocator is registered,
without adding a movement step or duplicate visit. At time zero the finding
robot adds the clue to belief, invokes its allocator observation hook, publishes
the clue, and then publishes allocator output. Repeated reports of an already
searched peer cell update peer pose state but do not trigger another belief
normalization. Each run records both the source-file SHA-256 and a canonical
SHA-256 of the ordered selected scenarios. Pass that canonical value back with
`--expected-scenario-sha256 <hash>` on every algorithm/Top-K condition to fail
before execution if IDs, order, targets, or clues differ.

The `topk_filter` profile also creates or verifies the shared
`../results/bayesian_clue_search/primary_topk_campaign/scenario_manifest.json`
automatically. It records
the grid, logic revision, selection hash, and ordered trial IDs, so a later
algorithm or Top-K condition with a different selection is refused before any
trial runs. Use `--scenario-manifest-lock <path>` for a deliberately separate
study cohort. `--no-scenario-manifest-lock` is the explicit escape hatch for a
non-paired diagnostic; the `custom` profile remains unlocked by default.

For the repaired canonical `final_trial_500.csv` currently committed here:

- raw file SHA-256:
  `9139f6a4fa259016f0e650489d605333758491b62151a742e406cc17dd5df085`
- canonical ordered selection SHA-256 for all 500 trials:
  `823213c90703fd83224ad7122ee730ba64af3769ea517af252103bddd907f681`
- canonical ordered selection SHA-256 for the first 300 trials:
  `33ddd00e9e07f86e272c4a946f91c9a9c4ee08ae6e902309b63caa0c5a8d5fa4`

The selection hash, rather than only the raw file hash, is the cross-condition
identity: it covers ordered trial IDs, targets, and clues after `--max-trials`
is applied.

Protected collision messages retain the sender's current cell and intended
next cell as separate values. Delivered normal-state positions block route
planning; protected current cells and intents both participate in the immediate
movement-safety check. A robot publishes one protected current-cell/no-next
clear the first time it has no goal or route; unchanged later clears are
deduplicated.

CBAA release messages clear a table entry only when the released winner
matches and the stored bid is no greater than `released_bid + 1e-9`; an absent
or `NO_BID` released bid is not a wildcard. DGA publishes only owner prefixes
whose first three cells changed, sends one empty clear when a previously sent
owner disappears, and preserves omitted unchanged owner prefixes at receivers.

`computational_performance.csv` reports candidate-filter time,
allocator-solve time excluding that filter work, and end-to-end allocator time.
The solve-only and end-to-end allocator columns include total, mean, median,
95th-percentile, and maximum host latency. Allocator percentage is reported
relative to host trial runtime.

Each headless run also writes `computational_performance.csv`. It contains
per-robot host-clock timing for allocator calls, including total, mean, median,
95th-percentile, and maximum latency plus the allocator share of total host
trial runtime. These measurements are host execution performance and do not
affect simulated mission time.

Headless trials and unit tests require Python 3.10 or newer and use only the
standard library. The live viewer additionally requires `pygame`.

The paired 500-trial Top-K campaign has a dedicated resumable launcher:

```powershell
python -m benchmark_sim.run_topk_campaign
```

It serially smoke-tests all 36 algorithm/Top-K conditions, uses 75% of the
available CPU cores by default, pins each single-threaded simulator worker to
one core, and schedules longest measured shards first. Every 15,000-event
first-pass shard finishes before failed trials are retried at 20,000 events.
Use `--smoke-only` to stop after validation and `--skip-smoke` to resume from
an existing passing smoke report.

Run the reproducible behavior-reference-versus-default-DGA timing and
allocation benchmark with:

```powershell
python benchmark_sim/tests/dga_optimization/benchmark_dga_optimized.py
```

Run the archived/reference-versus-default ACBBA, CBAA, HIPC, and PI memory and
timing comparison with:

```powershell
python -m benchmark_sim.tests.benchmark_allocator_memory_optimization
```
