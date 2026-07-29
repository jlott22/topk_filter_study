# Decentralized Allocator Optimization and Equivalence Log

## 2026-07-26 — Final 500-trial Top-K simulation campaign

- Completed 18,000 condition-trials: six allocation algorithms (`ACBBA`,
  `CBAA`, `DGA`, `DMCHBA`, `HIPC`, and `PI`) at six Top-K levels (100%, 75%,
  50%, 25%, 10%, and 5%), using the same 500 scenarios in every condition.
- Used a 19 × 19 grid, four robots, ideal communication, commitment horizon
  three, and one single-threaded trial process per worker core.
- Ran structural and arithmetic smoke validation for all 36 conditions before
  the full campaign. The final output validation passed for all conditions.
- First-pass trials used a 15,000-event safety cap. Initial failures were
  retried after the campaign at 20,000 events, then individually at 50,000 and
  100,000 events.
- Five trials remained non-terminating and were excluded from descriptive
  aggregate metrics without imputation:

  | Algorithm | Top-K | Trial IDs | Excluded |
  |---|---:|---|---:|
  | HIPC | 5% | 249 | 1/500 |
  | HIPC | 10% | 81, 435 | 2/500 |
  | HIPC | 50% | 201 | 1/500 |
  | DGA | 50% | 235 | 1/500 |

- The five exclusions represent 0.028% of the 18,000 condition-trials.
  Investigation identified a rare deterministic collision-resolution
  livelock. After peer-prediction mismatches removed peers from HIPC or DGA
  local team planning, incompatible plans caused robots to alternate between
  adjacent cells. Successful sidesteps reset the blocked-goal failure history,
  preventing the normal backoff threshold from breaking the cycle. Raising the
  event cap prolonged the same state without gaining coverage.
- Counterfactual diagnostic runs that retained peers in local planning
  completed all five scenarios below the original 15,000-event cap. These
  diagnostics did not modify the campaign data or production source.
- Final descriptive results and the exclusion statement are consolidated in
  `results/bayesian_clue_search/primary_topk_campaign/README.md`.
  Condition-level means are stored in
  `results/bayesian_clue_search/primary_topk_campaign/final_condition_summary.csv`.

## Top-K filter testing plan

### Objective

Measure how reducing the Top-K candidate-filter rate affects search performance
and computational performance for ACBBA, CBAA, DGA, DMCHBA, HIPC, and PI.
Each reduced-rate condition will be compared with the 100% Top-K baseline for
the same algorithm.

### Experimental matrix

The six Top-K rates are:

| Top-K rate | Candidate limit on a 19 by 19 grid |
|---:|---:|
| 100% | 361 |
| 75% | 271 |
| 50% | 181 |
| 25% | 90 |
| 10% | 36 |
| 5% | 18 |

The configured candidate limit is a ceiling. The actual retained count may be
lower when fewer eligible cells remain and should be recorded when possible.

Simulator:

- 6 algorithms by 6 Top-K rates;
- 300 trials per condition;
- 10,800 total condition-trials;
- the same 300 scenario IDs and random seeds used for every condition.

Hardware:

- 6 algorithms by 6 Top-K rates;
- 5 matched physical scenarios per condition;
- 180 total team trials;
- the same five scenarios, starting configuration, and target/clue layout used
  across all conditions.

### Primary search-performance outcomes

- successful target completion and failure rate;
- `total_team_steps`;
- `max_steps_any_robot`;
- change and percentage change from the algorithm's 100% Top-K baseline.

Hardware will additionally use hub-recorded `duration_s` as the primary
real-time search metric. Mean steps per agent will be reported as
`total_team_steps / robot_count`, but will be interpreted as workload rather
than elapsed search time.

Timeouts, non-progress failures, memory failures, invalid target reports, and
incomplete trials will be reported explicitly rather than discarded or treated
as ordinary completed trials.

### Computational-performance outcomes

Simulator:

- host trial runtime;
- allocator call count;
- allocator total, mean, median, p95, and maximum call time;
- allocator percentage of host trial runtime;
- pre-clue and post-clue allocator totals.

Hardware, per robot and aggregated across the team:

- candidate-filter call count, total time, maximum time, and mean time;
- allocator solve time excluding candidate-filter time;
- total allocation-decision time;
- allocator call count, mean call time, and maximum call time;
- per-agent allocator time as a percentage of trial wall time;
- team-normalized allocator utilization:
  `sum(agent allocator time) / (robot count * trial wall time)`.

### Metric definitions and CSV traceability

Top-K configuration is recorded as both the requested rate and the rounded
candidate ceiling:

- `top_k_rate`;
- `top_k_max_cells`.

On a 19 by 19 grid, round-half-up produces candidate ceilings of 361, 271,
181, 90, 36, and 18 for the six study rates. The simulator accepts the nominal
rate through `--top-k-rate`, which prevents a requested rate such as `0.75`
from being reconstructed imprecisely as `271 / 361`.

The simulator writes both Top-K fields to `trial_summary.csv`,
`system_performance.csv`, `robot_performance.csv`, and
`computational_performance.csv`. Each Pololu writes both fields to its onboard
metrics row, and the hardware hub writes them to its team and per-robot summary
CSVs. A fresh hardware output file or directory must be used after a schema
change so rows are not appended beneath an older CSV header.

Simulator computational metrics are per robot and use host wall-clock timing:

- `allocator_calls`;
- allocator total, mean, median, p95, and maximum time;
- allocator percentage of host trial runtime;
- pre-clue and post-clue allocator totals;
- candidate-filter call count and total, mean, and maximum time.

The simulator candidate-filter timer covers the complete candidate operation:
discovery of eligible cells, ranking, and Top-K truncation. Candidate-filter
time is a subset of end-to-end allocator time and must not be added to allocator
time when calculating total load.

Each Pololu onboard row records:

- `trial_time_ms`;
- `mean_step_time_ms = trial_time_ms / steps`;
- `candidate_filter_calls`;
- candidate-filter total, mean, and maximum time in microseconds;
- allocator solve time excluding measured candidate-filter time;
- `allocator_calls`;
- allocator total, mean, and maximum time in microseconds;
- `allocator_time_pct =
  allocator_time_us_total / (trial_time_ms * 1000) * 100`.

Zero-call, zero-step, and zero-duration divisions produce `0.0`. Pololu
allocator calls are counted in the same `finally` block that records elapsed
allocator time, so attempted calls that raise an exception remain represented.
Simulator allocator calls use the equivalent `finally`-based timing around
every `allocator.choose_goal()` call.

The legacy `busy_ms` and `compute_time_ms` columns are not exported. The
`busy_ms` counter remains internal only because it feeds the separate legacy
`cpu_util_pct` field.

The current hardware `cpu_util_pct` is instrumented main-loop activity, not a
complete measurement of RP2040 processor utilization. It can include blocking
waits and does not independently account for the second `_thread`. Likewise,
`gc.mem_free()` and `mem_free_min` describe sampled MicroPython GC-heap
headroom, not total SRAM, per-core stacks, static firmware storage, or
necessarily the shortest transient allocation peak.

If stronger CPU or memory claims become necessary, the preferred follow-up is
exclusive per-phase timing for both threads, GPIO/logic-analyzer validation,
pre/post-GC checkpoint measurements, a between-trial largest-contiguous-block
fragmentation diagnostic, and only then custom MicroPython firmware for true
idle, allocator, and stack high-water accounting. These deeper profiling
changes have been proposed but are not implemented in the current study code.

### Analysis and controls

The scenario or physical trial is the experimental unit. Allocator calls and
individual robot records are nested observations and will not be treated as
independent trials.

For each algorithm and Top-K rate, report the trial distribution, mean, median,
maximum, p95 where appropriate, and 95% confidence intervals. Comparisons with
the 100% condition will use scenario-paired absolute differences and percentage
differences. Any correction used for multiple Top-K comparisons will be stated.

Simulator software revision, scenario file and checksum, seed, grid size,
robot count, starting layout, communication model, and all non-Top-K parameters
will remain fixed and be recorded.

Hardware condition order will be randomized or counterbalanced. Robot identity,
starting position, calibration, battery state, firmware revision,
communication configuration, and environment will be recorded. Hardware
results based on five matched trials will be treated as pilot/descriptive
evidence unless the trial count is increased following a variance or power
assessment.

## Purpose

This is the repository-wide, append-only record for simulator and hardware
optimization work across all allocation algorithms. It tracks active and
archived files, behavioral requirements, implementation changes, memory and
runtime tradeoffs, validation evidence, deployment status, and remaining
simulator-to-hardware differences.

Detailed records should be kept under the appropriate algorithm heading.
Important changes should also receive a dated entry in the **Timestamped
updates** section at the bottom.

## Algorithm status index

| Algorithm | Optimization record | Active-version status | Equivalence status | Next recorded action |
|---|---|---|---|---|
| ACBBA | Documented below | Corrected simulator reference and memory-bounded hardware version are active defaults | Stateful desktop replay matches candidates, decisions, state, ownership changes, collision triggers, and messages at all six Top-K levels | Run physical precision, memory, and smoke gates |
| CBAA | Documented below | Corrected simulator reference and memory-bounded hardware version are active defaults | Stateful desktop replay matches candidates, decisions, state, ownership changes, collision triggers, and messages at all six Top-K levels | Run physical precision, memory, and smoke gates |
| DGA | Documented below | Corrected simulator reference and compact hardware version are active defaults | Seeded 25-generation output, prediction behavior, and dedicated-RNG isolation match on desktop | Run physical precision, memory, and smoke gates |
| DMCHBA | Documented below | Corrected simulator reference and virtual-matrix hardware version are active defaults | Virtual and dense matrices, assignments, ties, pseudotasks, and routes match on desktop | Run physical precision, memory, and smoke gates |
| HIPC | Documented below | Corrected simulator reference and memory-bounded hardware version are active defaults | Stateful desktop replay matches candidates, decisions, state, ownership changes, collision triggers, and messages at all six Top-K levels | Run physical precision, memory, and smoke gates |
| PI | Documented below | Corrected simulator reference and memory-bounded hardware version are active defaults | Stateful desktop replay matches candidates, decisions, state, ownership changes, collision triggers, and messages at all six Top-K levels | Run physical precision, memory, and smoke gates |

“Not yet documented” does not imply that an algorithm has no changes or tests;
it means this log has not yet established and verified its baseline.

## Logging and validation standard

Each algorithm record should include:

1. Active simulator and hardware paths.
2. Archived baseline paths.
3. Algorithm parameters and candidate-filter configuration.
4. Identified failure, bottleneck, or motivation.
5. Representation and execution changes.
6. Explicit behavior-preservation argument.
7. Memory and runtime measurements with platform identified.
8. Exact test command and pass count.
9. Production-size and deterministic-tie coverage.
10. Hardware deployment results and remaining risks.
11. Simulator-to-hardware differences.
12. A timestamped summary entry at the bottom of this file.

Keep these claims distinct:

- archived original versus optimized simulator equivalence;
- archived original versus optimized hardware equivalence;
- optimized simulator versus optimized hardware equivalence;
- physical trial validation.

Passing one level does not establish the others.

## Cross-cutting implementation records

### Top-K filtering semantics and placement

Top-K filtering is implemented for ACBBA, CBAA, DGA, DMCHBA, HIPC, and PI in
both simulator and hardware paths. A configured rate is converted from the
total grid size with round-half-up; on a 19 by 19 grid the study conditions are
361, 271, 181, 90, 36, and 18 cells for rates 100%, 75%, 50%, 25%, 10%, and
5%, respectively.

Each allocator receives only the filtered eligible task set. Filtering occurs
before allocation, so an allocator cannot inspect the complete eligible grid
unless Top-K is `1.0`. The actual result can be smaller than the configured
limit when cells are already searched, invalid, or otherwise unavailable.
When truncation is needed, valid cells are ranked by descending target
probability, ascending Manhattan distance from the selecting robot, and then
ascending `(x, y)`. When the eligible count is at most K, ACBBA, CBAA,
DMCHBA, and PI preserve their original y-major scan order. DGA and HIPC remain
probability-ranked because those algorithms rank before filtering. Distance is
a secondary ranking key, not part of the retained probability value.

The hardware initially used a flashed `TOP_K_PERCENT` and derived
`TOP_K_MAX_CELLS`. It now defaults to `1.0` and accepts the acknowledged
per-trial runtime configuration documented below.

### ESP32 UART/MQTT compatibility audit

The runtime configuration protocol is compatible with the existing
`hardware/esp32_DTA_BENCHMARK/esp32_DTA_BENCHMARK.ino`; no mandatory firmware
change was made. The bridge already:

- subscribes to shared MQTT `hub/command` and emits
  `997.<payload>-` to the Pololu as UART topic 7;
- accepts Pololu UART topic 6 and publishes it on MQTT `<robot_id>6`;
- uses 115200 baud and a 1,024-byte UART receive buffer.

The complete configuration and acknowledgment frames remain below the Pololu
256-byte per-frame parser limit. Protocol fields contain no `-`, so the
delimiter remains unambiguous. Peer ESP32s also forward topic-6
acknowledgments to their Pololus; inbound topic 6 is control-only and is
ignored by allocator handlers.

All six Pololu programs now request 4,096-byte RX and 1,024-byte TX driver
buffers, serialize shared-buffer construction and complete writes under one
lock, retry short/zero/`None` writes to a deadline, and parse RX streams in
bounded 256-byte chunks with oversize-frame discard and delimiter
resynchronization. Counters and deduplication state commit only after a
successful complete write. Malformed topic-6 payloads are audited and cannot
enter trial or allocator-message metrics.

Control uses sequenced, idempotent application acknowledgments:

```text
PRESTART/READY -> START/STARTED -> RUN/RUNNING
```

Every robot is stationary and armed before `RUN`; the initial `RUN` publish is
the hub time-zero boundary. Armed robots retain peer traffic while a delayed
`RUN` is retried, and a target before full `RUNNING` quorum invalidates the
trial. Physical heap headroom with the larger buffers, live burst behavior,
and end-to-end QoS-0 delivery of protected target/intent messages remain bench
gates. Arduino CLI and PlatformIO were unavailable for a local ESP32 firmware
compile, so the deployed ESP32/Pololu/MQTT chain must still be validated.

## Algorithm records

### ACBBA, CBAA, HIPC, and PI — shared memory optimization

The four paired allocators now use memory-bounded implementations at their
standard simulator and hardware paths.

Active simulator defaults:

- `simulator/benchmark_sim/algorithms/ACBBA.py`
- `simulator/benchmark_sim/algorithms/CBAA.py`
- `simulator/benchmark_sim/algorithms/HIPC.py`
- `simulator/benchmark_sim/algorithms/PI.py`
- shared primitives:
  `simulator/benchmark_sim/algorithms/memory_optimized.py`

Archived simulator baselines:

- `archive/ACBBA_simulator_unoptimized.py`
- `archive/CBAA_simulator_unoptimized.py`
- `archive/HIPC_simulator_unoptimized.py`
- `archive/PI_simulator_unoptimized.py`

Active hardware defaults:

- `hardware/Pololu_ACBBA.py`
- `hardware/Pololu_CBAA.py`
- `hardware/Pololu_HIPC.py`
- `hardware/Pololu_PI.py`
- required shared device module: `hardware/allocator_memory.py`

Archived hardware baselines:

- `archive/Pololu_ACBBA_unoptimized.py`
- `archive/Pololu_CBAA_unoptimized.py`
- `archive/Pololu_HIPC_unoptimized.py`
- `archive/Pololu_PI_unoptimized.py`

The archived files were copied before promotion and verified byte-for-byte by
SHA-256 against the pre-optimization defaults.

#### Shared representation changes

All four allocators now use the same cell identity and candidate priority:

```text
cell_id = y * grid_size + x
descending target probability
ascending Manhattan distance from the local robot
ascending x
ascending y
```

The simulator uses reusable unsigned-16-bit scan/ranking buffers with cached
numeric ranking keys. Hardware uses one unsigned-16-bit Top-K buffer and
decodes `(x, y)` only while iterating selected candidates. At Top-K 271, the
hardware candidate-ID payload is 542 bytes.

Winner/owner, bid/significance, timestamp, pending-delta, and last-sent
cell-keyed dictionaries were replaced where applicable by fixed cell-indexed,
mapping-compatible tables. An active flag preserves the distinction between
an absent entry and a stored no-owner/no-bid/no-time value. Hardware numeric
fields use signed 64-bit typed arrays, preserving the existing sentinel range.

This reserves bounded storage at initialization and can use more retained
memory than a nearly empty dictionary. It prevents later dictionary growth,
duplicate tuple keys, and heap fragmentation; therefore cold peak allocation
and repeated-trial stability are the important measures.

#### Algorithm-specific changes

ACBBA computes Top-K once per bundle refill, evaluates insertion distance
without temporary path slices, retains separate path/bundle semantics, and
emits pending messages in the same path-first then `(x, y)` order without a
temporary ordered list and set. Queued bundle metadata remains a snapshot
taken when the delta is created.

CBAA uses fixed winner/bid, pending, and last-sent tables. Pending messages and
invalid entries are found by scanning canonical cell IDs rather than copying
and unioning dictionary keys. Single-assignment selection, releases, and
payload order are unchanged.

HIPC uses fixed unsigned cell-ID plan storage, route-length bytes, endpoints,
and an assigned-cell bytearray for the team TAA. The probability normalizer is
computed once for team planning and local bundle replacement. The same public
team-plan dictionary is reconstructed after the bounded solve.

PI computes Top-K and its probability normalizer once per bundle refill.
Inserted and removed routes are evaluated without temporary path lists while
retaining the original route-cost addition order. Marginal significance,
timestamps, candidate ties, and path messages are unchanged.

#### Simulator validation and measured tradeoff

- Equivalence tests:
  `simulator/benchmark_sim/tests/test_allocator_memory_optimized_equivalence.py`
- Benchmark:
  `simulator/benchmark_sim/tests/benchmark_allocator_memory_optimization.py`
- Recorded comparison:
  `results/bayesian_clue_search/allocator_memory_optimization.md`

Coverage includes randomized finite and uniform probabilities, one through
four robots, candidate limits `all`, 5, 12, and 25, production 19 by 19
Top-K=271, second reallocations, exact state/message equality, and complete
four-robot event traces. The complete simulator suite passed 114 tests.

In the recorded CPython production comparison, candidate-call traced peak
allocation fell from 63,924 bytes to 18,112 bytes for ACBBA, CBAA, and PI, and
from 94,332 bytes to 18,112 bytes for HIPC. Cold first-allocation peak fell by
approximately 31% for ACBBA, 37% for CBAA, 59% for HIPC, and 45% for PI.

The bounded Python selector was slower than CPython's C-level tuple sort:
candidate filtering was approximately 5.6-7.9 times slower and first
allocation approximately 1.4-6.1 times slower, depending on the algorithm.
This is an accepted memory-for-time tradeoff pending physical RP2040 timing.

Run from `simulator/`:

```text
python -m unittest discover -s benchmark_sim/tests -p "test_*.py"
python -m benchmark_sim.tests.benchmark_allocator_memory_optimization
```

#### Hardware validation and deployment

`hardware/test_allocator_memory_optimized_equivalence.py` compares each active
script with its archived original without importing Pololu drivers. Coverage
includes randomized Top-K 20-100%, production Top-K 271, existing peer claims,
paths, consensus tables, and allocation-payload order. The complete hardware
desktop suite passed 17 tests.

Run from the repository root:

```text
python -m unittest discover -s hardware -p "test_*.py"
```

Physical deployment is not yet validated. `allocator_memory.py` must be copied
beside the ACBBA, CBAA, HIPC, or PI program on every robot. Record
`gc.mem_free()` before and after the first post-clue allocation, minimum free
heap over repeated reallocations, allocation duration, and UART/timing effects.

This establishes archived-original-versus-optimized equivalence independently
within simulator and hardware. It does not resolve the previously identified
simulator-to-hardware scoring, normalization, fixed-point, epsilon, or sentinel
differences. Those need a separate parity pass before claiming exact
simulator-to-hardware equivalence.

### DGA — scope, implementation, and validation

The compact-population DGA implementations now occupy both standard paths:

- Active simulator default: `simulator/benchmark_sim/algorithms/DGA.py`
- Archived simulator baseline: `archive/DGA_simulator_unoptimized.py`
- Active hardware default: `hardware/Pololu_DGA.py`
- Archived hardware baseline: `archive/Pololu_DGA_unoptimized.py`
- Simulator equivalence tests:
  `simulator/benchmark_sim/tests/test_dga_optimized_equivalence.py`
- Simulator benchmark:
  `simulator/benchmark_sim/tests/dga_optimization/benchmark_dga_optimized.py`
- Simulator-to-hardware core tests:
  `hardware/test_dga_simulator_equivalence.py`

Both active implementations retain the 30-member population, 25 generations,
two elites, three-way tournament selection, random-segment crossover, five
mutation operators, max-normalized probability objective, preparation source
order, repair semantics, stable solution ordering, and commitment behavior.
Population plans use flat unsigned 16-bit cell IDs plus unsigned 16-bit route
lengths. They are unpacked before crossover and mutation, so those operators
still act on the same route dictionaries and consume random choices in the
same order. Preparation streams only the best 30 plans, packed plans are
ranked directly, and canonical children skip a redundant final repair.

Before memory optimization, the hardware DGA was explicitly aligned to the
simulator in population size, generation count, crossover, all five mutation
operators, max-normalized probability cost, population preparation, received
solution handling, and final solution lifecycle. Platform RNG implementation
was intentionally allowed to differ. The compact representation pass then
preserved those aligned semantics while removing population-object overhead.

Seeded simulator tests include randomized grid/team cases, repeated
reallocation, exact 30-population/25-generation behavior, and a 19 by 19
Top-K=271 case. Hardware extraction tests cover edge costs, all crossover and
mutation operators, preparation, one-generation evolution, and a complete
25-generation search. The host simulator benchmark measured approximately
1.9-2.4 times lower runtime and 3.5-10.1 times lower traced peak allocation
over tested Top-K limits 18 through 361. These CPython measurements do not
replace physical RP2040 heap and timing tests.

Recorded first-allocation comparison:

| Configured Top-K | Effective candidates | Runtime speedup | Peak-allocation reduction |
|---:|---:|---:|---:|
| 18 | 18 | 1.94x | 3.51x |
| 36 | 36 | 2.12x | — |
| 48 | 48 | 2.15x | 5.22x |
| 72 | 72 | 2.20x | — |
| 90 | 90 | 2.20x | 6.63x |
| 181 | 181 | 2.29x | 8.29x |
| 271 | 271 | 2.32x | 9.25x |
| 361 | 359 | 2.40x | 10.06x |

Every measured case produced the same ordered candidates, best plan, goal,
fitness, and final population as the reference simulator allocator.

### DMCHBA — scope and status

This log documents the completed memory optimization and promotion of both the
RP2040/MicroPython and simulator DMCHBA allocators. Both original versions are
archived, and the optimized versions now occupy the standard/default paths.

Hardware:

- Original archive: `archive/Pololu_DMCHBA_unoptimized.py`
- Active optimized default: `hardware/Pololu_DMCHBA.py`
- Desktop parity tests: `hardware/test_dmchba_optimized_equivalence.py`

Simulator:

- Original archive: `archive/DMCHBA_simulator_unoptimized.py`
- Active optimized default: `simulator/benchmark_sim/algorithms/DMCHBA.py`
- Parity tests:
  `simulator/benchmark_sim/tests/test_dmchba_optimized_equivalence.py`
- Standard loader:
  `benchmark_sim.algorithms.DMCHBA:DMCHBAAllocator`
- Compatibility aliases: `DMCHBAOptimizedAllocator` and `Allocator`

The hardware and simulator optimizations were validated independently against
their respective archived originals. This isolates memory-representation
changes from the separate simulator-to-hardware behavior-alignment task.

### DMCHBA — simulator optimization and promotion

A later, separate pass archived the original simulator allocator and promoted
the optimized implementation to the default module:

- Original simulator archive: `archive/DMCHBA_simulator_unoptimized.py`
- Active optimized default: `simulator/benchmark_sim/algorithms/DMCHBA.py`
- Simulator parity tests:
  `simulator/benchmark_sim/tests/test_dmchba_optimized_equivalence.py`

The simulator duplicate uses reusable linear Hungarian workspaces, an
agent-by-task base-cost cache, implicit clone rows and pseudotask columns, a
virtual cost matrix, direct collection of only the local robot's assignments,
and commitment-prefix-only route ordering. It preserves the simulator's
floating-point objective, normalization, tie terms, Hungarian traversal order,
trigger logic, debug output, and goals.

Parity coverage includes randomized grids, one through four agents, candidate
limits, uniform tie cases, collision and path-exhaustion sequences, exact
dense-versus-virtual assignment equality, a 19 by 19 Top-K=271 case, and the
19 by 19 all-candidate default. The focused and existing DMCHBA-related test
run completed with 17 passing tests. In the all-candidate case, the optimized
numeric workspace plus transient agent-by-task cache was below 32 KiB, versus
more than 1 MiB for double-precision dense numeric payload alone.

This validates archived-original-simulator versus active-optimized-simulator
equivalence. It does not yet validate simulator versus physical-hardware
equivalence.

Run the simulator parity and existing DMCHBA suites from `simulator/` with:

```text
python -m unittest benchmark_sim.tests.test_dmchba_optimized_equivalence benchmark_sim.tests.test_dmchba_integration benchmark_sim.tests.test_clue_replan_triggers benchmark_sim.tests.test_probability_cost_consistency -v
```

### DMCHBA — original hardware failure mechanism

At `GRID_SIZE = 19` and `TOP_K_PERCENT = 0.75`, the allocator retains 271
candidate cells. With four agents, matching-by-clone creates 272 clone rows and
a logical 272 by 272 Hungarian matrix containing 73,984 costs.

Even an ideal packed signed-32-bit matrix would require 295,936 bytes
(approximately 289 KiB). This already exceeds the RP2040's total SRAM before
accounting for MicroPython, bytecode, stacks, robot drivers, maps, UART buffers,
candidate objects, clone metadata, or Hungarian work arrays. The original
nested Python lists require substantially more memory than this lower bound.

Garbage collection cannot solve this capacity problem. The original script
already calls `gc.collect()` immediately before allocation, but the requested
dense object is larger than the available memory.

### DMCHBA — implemented hardware optimization

#### 1. Virtual clone matrix

The optimized solver no longer constructs the square list-of-lists matrix.
Hungarian still operates on the same logical rows, columns, and costs, but a
cost is obtained from compact state when the solver needs it.

This preserves:

- candidate cells and their ordering;
- number and ordering of agent clones;
- pseudotask columns;
- original scaled-integer cost calculation;
- clone bias;
- Hungarian traversal and comparison order;
- resulting row-to-column assignment.

#### 2. Agent-by-task base-cost cache

Clone rows belonging to the same agent share the same agent-to-task base cost.
Only the small agent-by-task table is stored:

```text
number of agents * number of retained tasks
```

At 75% Top-K this is approximately `4 * 271 = 1,084` signed-32-bit entries,
instead of 73,984 expanded matrix entries. Clone bias is added when a logical
matrix value is requested.

#### 3. Packed candidate cells

Candidate cells are represented internally as unsigned integer cell IDs rather
than `(x, y)` tuples. Coordinates are decoded only when required.

The optimized Top-K selector uses a fixed unsigned-16-bit buffer and preserves
the original ranking:

```text
descending target probability,
ascending Manhattan distance from this robot,
ascending (x, y)
```

It retains only the current Top-K entries and avoids the original list of
nested ranking tuples, float objects, cell tuples, full sort result, and copied
output list.

#### 4. Fixed reusable Hungarian workspaces

Hungarian work memory is allocated once and reused:

- signed-64-bit arrays for dual potentials and minimum reduced costs;
- unsigned-16-bit arrays for matches and predecessor indices;
- a bytearray for visited-column flags;
- a signed-16-bit row-to-column result buffer.

Signed 64-bit storage is required for behavioral parity because the
1,000,000,000 pseudotask cost can drive intermediate dual potentials beyond
signed-32-bit range. An initial signed-32-bit implementation was rejected by
the production-size parity test after it overflowed.

#### 5. Implicit clone and column metadata

The optimized version does not allocate:

- a tuple for every clone row;
- a copied task-column list;
- explicit `None` pseudotask entries.

Agent index and clone index are derived from the logical row number.
Pseudotasks are identified from column indices beyond the real task count.

#### 6. Commitment-prefix construction

The original allocator greedily ordered every cell assigned to the local
robot, then retained only the first three. The optimized version performs the
same greedy selection but stops after producing the three observable committed
cells.

Later greedy choices cannot change an already selected prefix, so this reduces
work and temporary storage without changing the executed path.

### DMCHBA — resulting memory behavior

For the 19 by 19, four-agent, 75% Top-K configuration:

- logical matrix: 272 by 272;
- original packed-matrix lower bound: approximately 289 KiB;
- optimized fixed typed-workspace payload: less than 16 KiB;
- structural reduction: greater than 20 times relative to a packed dense
  matrix, and much larger relative to nested Python lists.

A CPython diagnostic measured approximately 3,074 KiB of transient allocation
for the original solve and approximately 1.6 KiB of transient allocation for
the optimized solve after its fixed workspace had been allocated. CPython
object sizes are not RP2040 measurements, but this verifies that quadratic
temporary allocation was removed rather than relocated.

### DMCHBA — hardware behavioral validation

The desktop test harness extracts only allocator functions from the hardware
scripts. This avoids importing Pololu modules, initializing hardware, starting
threads, or launching a robot mission.

Tests currently verify:

- exact ordered Top-K candidate equality;
- exact virtual-versus-dense cost equality;
- exact three-cell committed-path equality;
- one through four known agents;
- randomized belief maps;
- uniform-probability deterministic ties;
- searched cells and obstacles;
- Top-K fractions from 20% through 100%;
- twenty randomized small-grid cases;
- a production-size 19 by 19, four-agent, 75% Top-K case;
- linear workspace size rather than quadratic matrix growth.

All tests pass. In the production-size fixture, both implementations returned:

```text
[(13, 16), (13, 15), (12, 15)]
```

Run the parity suite from the repository root with:

```text
python -m unittest hardware.test_dmchba_optimized_equivalence -v
```

### DMCHBA — accepted runtime tradeoff

The optimized interpreted implementation is slower because it trades dense
matrix caching for compact cost reconstruction and typed-array access.

In one desktop production-size fixture:

- original dense solver: approximately 1.21 seconds;
- optimized virtual solver: approximately 2.31 seconds.

These are CPython measurements, not RP2040 predictions. The original version
is faster only when sufficient memory exists to build its matrix; on the
RP2040 it fails before completing. The optimized version prioritizes a
completable solve with bounded memory.

If runtime later becomes unacceptable, the preferred improvement is native
compilation of the same virtual Hungarian loop. That would be an execution-
backend optimization and should retain the same candidate set, logical costs,
tie behavior, assignment, and committed path.

### DMCHBA — remaining hardware validation

Desktop parity does not replace testing on the Pololu robot. Before replacing
the original deployment script:

1. Flash `hardware/Pololu_DMCHBA.py` as the robot program.
2. Record `gc.mem_free()` immediately before and after the first allocation.
3. Confirm the first 75% Top-K solve completes without `MemoryError`.
4. Measure allocator duration and confirm UART buffering remains adequate.
5. Run repeated trials to check fragmentation and stable workspace reuse.
6. Repeat on all four robots with their deployment-specific robot IDs.

Firmware freezing and native compilation were not performed in this pass.
Those require the actual MicroPython/Pololu firmware build and should be
treated as optional deployment optimizations after functional board testing.

### DMCHBA — simulator-to-hardware equivalence follow-up

The memory representation changes have now been applied independently to both
platforms. Representation parity alone does not resolve existing behavioral
differences between the simulator and hardware algorithms. Before claiming
exact simulator-to-hardware equivalence, align and test:

- identical absolute Top-K value;
- probability normalization and coefficient;
- fixed-point scaling and rounding;
- cell and clone tie-breaking;
- later-clue reassignment behavior;
- collision-triggered reassignment;
- empty-assignment and nearest-cell fallback behavior;
- assignment-input signatures;
- commitment horizon.

A defensible final claim would be:

> The simulator and hardware use the same DMCHBA candidate ranking, fixed-point
> cost specification, clone construction, virtual Hungarian assignment,
> deterministic tie rules, reassignment triggers, and commitment horizon.
> Platform-specific differences are limited to sensing, communication, motion,
> profiling, and memory-storage backends.

Until those behavioral items are aligned, the completed tests support two
narrower claims:

1. The active optimized hardware allocator is behaviorally equivalent to the
   archived original hardware allocator for the tested inputs.
2. The active optimized simulator allocator is behaviorally equivalent to the
   archived original simulator allocator for the tested inputs.

## Future algorithm record template

Copy this structure beneath **Algorithm records** when starting another
algorithm:

```text
### ALGORITHM — scope and status
- Active simulator:
- Active hardware:
- Archived simulator:
- Archived hardware:
- Configuration:

### ALGORITHM — problem or motivation
- Failure/bottleneck:
- Reproduction conditions:
- Baseline measurements:

### ALGORITHM — implemented changes
- Change:
- Memory/runtime effect:
- Why behavior is preserved:

### ALGORITHM — validation
- Test files:
- Test command:
- Passed cases:
- Production-size result:
- Known gaps:

### ALGORITHM — deployment and follow-up
- Hardware result:
- Simulator-to-hardware differences:
- Next action:
```

## Timestamped updates

### 2026-07-24 — Added Top-K and computation-performance instrumentation

- Applied the same probability-first Top-K prefilter to all six simulator and
  hardware allocators before their allocation processes.
- Standardized rate-to-cell conversion with round-half-up and recorded both
  the requested rate and resulting cell limit.
- Added candidate-filter and complete/solve-only allocator timing to every
  Pololu metrics row, including calls, totals, means, maxima, and allocator
  trial-time percentage.
- Removed `busy_ms` and `compute_time_ms` as exported columns while retaining
  the internal busy counter solely for legacy CPU-utilization output.
- Added host-clock allocator and candidate-filter samples to the simulator and
  the separate `computational_performance.csv` export. These values measure
  host execution rather than simulated time.
- Added tests for known 19 by 19 Top-K conversions, onboard fields, simulator
  aggregation statistics, CSV export, and failed-trial replacement behavior.

### 2026-07-24 — Verified ESP32 transport compatibility

- Traced the new configuration and acknowledgment frames through hub MQTT,
  ESP32 topic-7/UART forwarding, Pololu parsing, Pololu topic-6 UART output,
  ESP32 robot-specific MQTT publication, and hub acknowledgment matching.
- Confirmed existing topic routing, delimiters, baud rate, and buffer sizes are
  compatible; no mandatory ESP32 source change is required.
- Recorded that configuration retries tolerate ordinary QoS-0 loss and that
  peer-forwarded configuration acknowledgments are ignored safely by Pololu.
- Recorded remaining risks and the physical bench-validation plan. The ESP32
  sketch was not locally compiled because Arduino CLI and PlatformIO were not
  installed.

### 2026-07-24 — Added acknowledged per-trial hardware conditions

- Area: hardware control, allocator memory, communication, metrics, tests, and
  documentation.
- Primary files:
  - `hardware/metrics_hub.py`;
  - all six active `hardware/Pololu_*.py` allocator programs;
  - `hardware/test_runtime_trial_configuration.py`;
  - `hardware/test_topk_metrics_logging.py`;
  - `hardware/README.md`.
- Changed the hardware default Top-K rate to `1.0`, corresponding to 361 cells
  on the 19 by 19 grid. The default drop rate is `0.0`.
- Added interactive per-trial Top-K and drop-rate prompts to the metrics hub,
  with Enter reusing the previous condition. Decimal rates are converted to
  integer millionths and Top-K cells use round-half-up.
- Added the idle-only configuration command:

  ```text
  CFG,<sequence>,<top_k_ppm>,<top_k_cells>,<drop_ppm>
  ```

- Added topic 6 robot acknowledgments:

  ```text
  CFGACK,<sequence>,<algorithm>,<top_k_ppm>,<top_k_cells>,<drop_ppm>,<status>
  ```

  Status is `OK`, `INVALID`, or `MEMORY_ERROR`. Repeated identical commands
  are idempotent. Stale acknowledgments are audited but cannot satisfy the
  current handshake.
- The hub now requires acknowledgments from every configured robot and verifies
  the flashed algorithm plus all applied settings before issuing pre-start or
  start. Missing, mismatched, or negative acknowledgments prevent the scenario
  from being consumed and prompt recovery/retry.
- Added exact-capacity allocator workspace rebuilding for all six hardware
  allocators:
  - ACBBA, CBAA, HIPC, and PI rebuild their packed candidate workspace;
  - DMCHBA rebuilds every candidate, agent-cost, Hungarian, and assignment
    workspace using the selected Top-K;
  - DGA updates its dynamic candidate bound while retaining grid-sized masks.
- Preserved the existing receiver-side message-loss policy. The configured
  drop rate updates each robot's `msg_drop_rate`; collision-intent and target
  alert topics remain protected.
- Added post-trial memory-error prompting, timeout/abort trial records, and
  command `3` to release robots from a timed-out or operator-aborted trial.
- Added robot-specific pre-trial terminal output for connection, home readiness,
  and acknowledged algorithm/Top-K/drop-rate settings. Active-trial console
  output suppresses routine state/allocation traffic and reports only clue and
  target detections.
- System and robot CSV rows now include `algorithm_verified`, `drop_rate`,
  compatibility `comm_level`, `top_k_rate`, `top_k_max_cells`,
  `memory_error`, `trial_status`, `failure_reason`, and `config_sequence`.
  Onboard rows include the robot-applied drop rate and configuration sequence.
  `<algorithm>_configuration_acks.csv` records the complete acknowledgment
  audit.
- Preserved noninteractive `--auto` operation with `--top-k-rate`,
  `--drop-rate`, and `--memory-error-default`. The old `--comm-level` option
  remains a compatibility alias for the drop rate.
- No ESP32 firmware change was required because the existing bridge already
  forwards hub topic 7 commands and robot topic 6 responses.
- Validation completed:
  - 34 hardware tests passed, including conversion, prompt, configuration,
    workspace-resize, memory-failure, stale/missing/mismatched acknowledgment,
    metrics, and allocator-equivalence coverage;
  - 114 simulator tests passed;
  - all changed Python files passed syntax checks;
  - `git diff --check` passed.
- Remaining work: deploy the updated hub and robot files, then physically test
  Top-K `0.05`, `0.75`, and `1.0` with drop rates `0.0` and `1.0`. Desktop
  tests do not establish RP2040 heap headroom, UART/MQTT timing, or physical
  recovery behavior.

### 2026-07-24 — Compact DGA promoted to both defaults

- Archived the prior simulator and Pololu implementations as
  `archive/DGA_simulator_unoptimized.py` and
  `archive/Pololu_DGA_unoptimized.py`.
- Promoted the validated compact simulator allocator to the standard
  `benchmark_sim.algorithms.DGA:DGAAllocator` loader.
- Ported packed 16-bit population storage, streaming preparation, direct
  compact ranking, reusable repair masks, and redundant-repair removal to
  `hardware/Pololu_DGA.py`.
- Confirmed seeded simulator equivalence and simulator-to-hardware core
  equivalence on desktop, including a complete 25-generation search.
- Physical RP2040 deployment, heap measurement, and timing remain outstanding.

### 2026-07-24 13:08:40 PDT — Log generalized for all algorithms

- Converted the DMCHBA-only document into the repository-wide allocator
  optimization and equivalence log.
- Added the algorithm status index for ACBBA, CBAA, DGA, DMCHBA, HIPC, and PI.
- Added a common validation standard and reusable future-algorithm template.
- Preserved the complete DMCHBA hardware and simulator optimization record.
- Confirmed the current DMCHBA file layout:
  - active hardware: `hardware/Pololu_DMCHBA.py`;
  - archived hardware baseline: `archive/Pololu_DMCHBA_unoptimized.py`;
  - active simulator: `simulator/benchmark_sim/algorithms/DMCHBA.py`;
  - archived simulator baseline: `archive/DMCHBA_simulator_unoptimized.py`.
- Recorded that both active DMCHBA implementations match their respective
  archived originals for the tested inputs, while simulator-to-hardware
  behavioral alignment remains pending.

### 2026-07-24 14:08:15 PDT — ACBBA, CBAA, HIPC, and PI optimization promoted

- Archived exact pre-optimization simulator and hardware defaults for all four
  algorithms.
- Promoted packed Top-K selection and fixed cell-indexed consensus tables to
  the standard simulator and hardware paths.
- Added reusable ACBBA/PI route evaluation and bounded HIPC team-plan storage.
- Preserved simulator decisions, state, messages, and complete multi-robot
  traces in randomized and production-size tests.
- Preserved hardware candidate order, paths, consensus tables, and allocation
  payload order in desktop extraction tests.
- Recorded substantially lower traced peak allocation with higher interpreted
  runtime.
- Verified 114 simulator tests and 17 hardware tests.
- Physical RP2040 heap/timing validation and simulator-to-hardware behavioral
  alignment remain outstanding.

### 2026-07-24 14:14:33 PDT - Final post-promotion validation

- Re-ran the complete test suites after the optimized allocators became the
  default implementations:
  - simulator: 114 tests passed;
  - hardware desktop-equivalence harness: 17 tests passed.
- Recompiled the shared memory modules and active ACBBA, CBAA, HIPC, and PI
  simulator and hardware files successfully.
- Confirmed the final modified-file set passes `git diff --check`.
- The production-size CPython benchmark measured cold allocation peak-memory
  reductions of approximately 31% for ACBBA, 37% for CBAA, 59% for HIPC, and
  45% for PI. Candidate-generation peak memory fell approximately 72% for
  ACBBA, CBAA, and PI and 81% for HIPC.
- Recorded the expected tradeoff: execution is slower and fixed cell-indexed
  tables can retain more memory in sparse cases, but transient peak allocation
  is substantially lower and memory demand is more predictable.
- The tested claim is exact old-to-optimized behavioral equivalence within
  each platform. This does not yet claim exact simulator-to-hardware
  equivalence, because pre-existing scoring and platform differences were not
  changed during this memory pass.
- Hardware deployment requires copying `hardware/allocator_memory.py` beside
  the selected `Pololu_*.py` allocator script.
- Physical RP2040 heap, timing, and mission tests remain the final deployment
  validation step.

### 2026-07-24 16:32:01 PDT - Top-K study plan and matched load metrics

- Added the repository-level Top-K experiment plan:
  - ACBBA, CBAA, DGA, DMCHBA, HIPC, and PI;
  - rates `1.0`, `0.75`, `0.50`, `0.25`, `0.10`, and `0.05`;
  - 300 matched simulator scenarios per algorithm/rate condition;
  - five matched physical scenarios per hardware condition;
  - 10,800 simulator condition-trials and 180 hardware team trials.
- Defined `total_team_steps`, `max_steps_any_robot`, completion/failure rate,
  and paired degradation from each algorithm's 100% Top-K condition as the
  primary search outcomes.
- Recorded the analysis controls discussed during review:
  - reuse the same simulator scenario IDs and seeds across all conditions;
  - treat the scenario or physical trial, not individual calls or robots, as
    the experimental unit;
  - report timeouts, non-progress, memory failures, invalid targets, and
    incomplete trials rather than silently dropping them;
  - randomize or counterbalance hardware condition order and record robot,
    calibration, battery, firmware, communication, start, and environment
    state;
  - treat five hardware trials as pilot/descriptive evidence unless a later
    variance or power analysis supports inferential claims.
- Added nominal `--top-k-rate` input to the simulator and retained the existing
  absolute `--max-candidate-cells` input as a mutually exclusive alternative.
- Added `top_k_rate` and `top_k_max_cells` to every simulator metric CSV.
  A real one-trial 75% run verified `0.75` and `271` in trial, system, robot,
  and computational outputs.
- Added the same two Top-K fields to all six Pololu onboard metric schemas and
  to hardware-hub team and per-robot CSVs. Added regression coverage across all
  six Pololu source schemas.
- Confirmed that the simulator already counted and timed every allocator call
  centrally. Added simulator timing for the complete candidate-filter
  operation across all six active algorithms, including packed optimized
  candidate generation.
- Added simulator candidate-filter call count and total, mean, and maximum
  host latency. Retained allocator total, mean, median, p95, maximum,
  pre/post-clue totals, and host-runtime percentage. Added
  `allocator_time_pct` as the cross-platform name while retaining
  `allocator_host_runtime_pct` for compatibility.
- Added the following to every Pololu allocator:
  - allocator call count;
  - allocator mean and maximum call time;
  - candidate-filter mean time;
  - trial wall time;
  - mean step time;
  - allocator percentage of trial wall time.
- Defined candidate-filter time as a subset of allocator time. Hardware
  `allocator_solve_time_us_total` continues to exclude separately measured
  candidate-filter time, while `allocator_time_us_total` remains the
  end-to-end allocation-decision total.
- Documented measurement limitations:
  - hardware CPU utilization is instrumented main-loop time rather than exact
    dual-core processor load;
  - sampled MicroPython heap headroom is not total SRAM or a guaranteed
    transient peak;
  - phase accounting, GPIO validation, fragmentation tests, and custom
    firmware remain optional future profiling improvements.
- Post-change validation completed:
  - 69 simulator integration and equivalence tests passed;
  - 7 focused simulator computational-metric tests passed, including direct
    coverage of all six active candidate filters;
  - 16 hardware metric and allocator-parity tests passed;
  - all six Pololu CSV schemas contain the required matched fields;
  - `git diff --check` passed.

### 2026-07-24 16:38:51 PDT - Conversation record reconciled

- Audited this log against the allocator-memory work and decisions from the
  conversation. Live simulation progress, process counts, CPU utilization,
  and completion estimates were intentionally excluded.
- Recorded the original DMCHBA RP2040 failure diagnosis: the cloned Hungarian
  problem created a quadratic dense Python-object matrix plus short-lived
  ranking, route, and solver allocations that exceeded or fragmented the
  available MicroPython heap.
- Recorded the behavior-preserving DMCHBA response: virtual clone costs,
  agent-by-task cost caching, packed candidates, fixed reusable Hungarian
  workspaces, implicit clone metadata, and bounded commitment-prefix
  construction. The slower interpreted runtime was accepted as the necessary
  tradeoff for a bounded solve that can complete within constrained memory.
- Recorded the promotion sequence used throughout the work: duplicate the
  simulator allocator, optimize the duplicate, compare it with the exact
  original, archive the original only after parity passes, promote the
  optimized version to the default loader, then archive and optimize the
  corresponding hardware implementation.
- Recorded the same completed workflow for ACBBA, CBAA, HIPC, and PI,
  including shared cell indexing, packed Top-K representations, reusable
  route evaluation, bounded HIPC plan storage, exact archived baselines, and
  the common `hardware/allocator_memory.py` deployment dependency.
- Recorded the compact DGA simulator and hardware promotion separately. DGA
  and DMCHBA were excluded from the later four-algorithm optimization pass
  because their memory work had already been completed.
- Confirmed that shared concepts use equivalent logical representations across
  algorithms where comparison requires it: probability-first Top-K ordering,
  deterministic distance/coordinate ties, packed cell IDs, fixed cell-indexed
  state, and reusable bounded workspaces. Simulator and MicroPython storage
  backends may differ when required by the platform.
- Preserved the final validation evidence for the four-algorithm memory pass:
  114 simulator tests, 17 hardware desktop-equivalence tests, successful
  compilation, and a clean `git diff --check`. Later metric-instrumentation
  validation is recorded in the immediately preceding timestamped entry.
- Preserved the measured memory/runtime tradeoff: substantially lower
  transient and cold-allocation peaks, slower interpreted execution, and
  potentially higher retained memory for sparse state because fixed tables
  are preallocated. This is intentional for predictable RP2040 allocation and
  reduced fragmentation.
- Locked the defensible equivalence claim: each active optimized simulator
  matches its own archived simulator baseline, and each active optimized
  hardware allocator matches its own archived hardware baseline for tested
  inputs. This supports calling each optimized implementation the same
  algorithm as its baseline.
- Did not claim universal simulator-to-hardware identity. Existing differences
  in scoring, normalization, fixed-point behavior, epsilon/sentinel handling,
  reassignment triggers, and platform I/O remain a separate alignment task
  unless an algorithm-specific cross-platform test explicitly establishes
  parity.
- Recorded that Top-K success thresholds discussed before implementation were
  engineering estimates rather than physical-board measurements. The
  optimized DMCHBA design and desktop production fixture cover the 19 by 19,
  four-agent, 75% Top-K case, but actual RP2040 success at each Top-K rate
  still requires heap, timing, fragmentation, UART, and repeated-mission
  measurements on all robots.

### 2026-07-24 22:55:52 PDT - UART hardening and true RUN release barrier

- Hardened all six Pololu transports with explicit 4,096-byte RX and
  1,024-byte TX driver buffers, bounded write-all handling for short/zero/None
  writes, one lock spanning shared frame construction and transmission, and a
  streaming 256-byte parser that discards oversize frames through the next
  delimiter.
- Replaced the two-stage release with sequenced
  `PRESTART/READY -> START/STARTED -> RUN/RUNNING` control:
  - `START` clears current-trial transport caches and resets onboard metrics
    exactly once, but keeps the robot stationary;
  - the hub waits for every `STARTED` acknowledgment before its initial `RUN`
    publish;
  - that initial `RUN` publish atomically establishes hub time zero and the
    pending-event boundary;
  - first receipt of `RUN` stamps the robot metric start time and releases
    motion without clearing belief, allocator state, peer state, or counters;
  - duplicate commands only re-acknowledge their already-applied transition.
- Armed robots now accept, apply, count, and forward peer state, intent,
  allocator, clue, and target traffic while RUN delivery is staggered.
  Non-target RUN-window events are replayed by the hub exactly once in arrival
  order after the full `RUNNING` quorum.
- A target received after preparation begins but before full `RUNNING` quorum
  invalidates the trial. A robot that learns the target while armed cannot
  subsequently begin motion. `ABORT` wakes and stops configured, ready, armed,
  or running robots.
- Reserved MQTT topic 6 strictly for configuration/control acknowledgments.
  Malformed topic-6 payloads are audited as invalid and cannot enter allocator
  message counts or trial events.
- Validation completed:
  - all 139 hardware desktop tests passed;
  - `mpy-cross` compiled `hardware/allocator_memory.py` and all six active
    `hardware/Pololu_*.py` programs;
  - AST comparison found one identical implementation across all six programs
    for the traffic gate, control state machine, and RUN wait loop;
  - every search loop was confirmed to leave metric reset at the first
    `START`, with no second reset when `RUN` wakes it;
  - `git diff --check` passed.
- Physical four-robot smoke trials and on-device K=361 memory-headroom
  measurements remain required before production data collection.

### 2026-07-24 - Simulation-first algorithm parity implementation

- This entry supersedes earlier historical notes that described
  simulator-to-hardware logic alignment as pending; those notes remain as a
  record of the state before this parity pass.
- Corrected and locked both simulator repositories first, then aligned all six
  active Pololu programs to that reference. The run provenance identifier is
  `dcta_parity_v1`.
- Standardized belief and allocation math:

  ```text
  w(c) = 0                                      when searched
       = 1                                      when no clue is known
       = sum_l 1 / (1 + Manhattan(c, l))        otherwise
  p(c) = w(c) / sum_u w(u), with uniform fallback when the sum is zero
  M    = max_u p(u), or 1 when no finite positive maximum exists
  q(c) = clamp(p(c) / M, 0, 1)
  J(a,c) = Manhattan(a,c) + 8 * (1 - q(c))
  B(a,c) = -J(a,c)
  ```

  Belief and `M` are fully refreshed after every accepted local or peer miss
  and clue mutation. Allocator tables and wire values use binary64,
  `EPS = 1e-9`, and round-trip-safe decimal encoding. Hardware fails startup
  if its scalar and `array('d')` behavior cannot preserve this contract.
- Locked Top-K conversion to
  `K = max(1, floor(361 * rate + 0.5))`, producing
  `361, 271, 181, 90, 36, 18`. Truncation ranks by descending `p`, ascending
  local Manhattan distance, then `(x,y)`. ACBBA, CBAA, DMCHBA, and PI retain
  y-major source order when no truncation is needed; DGA and HIPC remain
  probability-ranked.
- Aligned hardware starts, east headings, row bands, and complete cyclic
  serpentine sweeps with the simulator. Both simulators now observe every
  start cell at logical time zero after registration, including clue detection
  and normal clue publication, without adding a movement step or duplicate
  start visit.
- Enforced the `clue_search`, 19-by-19, four-robot configuration contract,
  horizon 3 except effective CBAA horizon 1, exact algorithm/revision fields,
  and an ordered scenario-manifest hash. Scenario validation is fail-fast for
  row count, IDs, integer bounds, clue count/uniqueness, target-clue overlap,
  and target-on-start. Trials 172, 203, and 494 were repaired in both canonical
  manifests.
- Verified matching scenario provenance:
  - raw 500-row file SHA-256:
    `9139f6a4fa259016f0e650489d605333758491b62151a742e406cc17dd5df085`;
  - ordered all-500 selection:
    `823213c90703fd83224ad7122ee730ba64af3769ea517af252103bddd907f681`;
  - ordered first-300 selection:
    `33ddd00e9e07f86e272c4a946f91c9a9c4ee08ae6e902309b63caa0c5a8d5fa4`.
- Aligned peer-miss deduplication, separation of droppable peer state from
  protected collision intent, one-time clue forwarding, independent
  receiver-side drop draws, goal persistence, allocator invocation cadence,
  clue-state retention, intent clear/deduplication, and terminal target
  accounting/freeze.
- Aligned A* independently of allocator scoring:

  ```text
  base = 1 + 0.3 * quarterTurns + 4 * searched(next)
  bonus = min(max(0, 5 * p(next)), max(0, base - 0.01))
  stepCost = max(0.01, base - bonus)
  g' = g + stepCost
  f  = g' + Manhattan(next, goal)
  ```

  Expansion is N/E/S/W with insertion-order heap ties and cell-keyed cost
  state. Delivered peer positions block the complete planned route. Collision
  handling retains the goal, first attempts an alternate route, and only after
  two protected conflicts clears it, temporarily invalidates it, waits a
  uniform 0-to-5-second collision-RNG delay, and raises the allocator collision
  trigger.
- Applied the algorithm-specific canonical equations and consensus behavior:
  - ACBBA uses insertion-distance marginal plus `8*(1-q)`, canonical
    bid/cell/index ties, Table-1 consensus, and suffix release;
  - CBAA uses `-J`, numeric robot-ID ownership ties, same-winner bid decreases,
    and guarded releases;
  - DGA uses the exact 30-member, 25-generation, two-elite GA with tournament
    3, crossover 0.7, mutation 0.3, five mutation operators, exact
    fitness/signature ties, changed owner prefixes/clears, corrected
    once-per-solution prediction assessment, and a dedicated
    CPython-compatible RNG isolated from packet loss and backoff;
  - DMCHBA uses the exact logical-clone dimensions, `J` objective,
    y-major/clone/row perturbation, pseudotasks, strict Hungarian comparisons,
    greedy three-cell route order, and exact input signature;
  - HIPC uses the canonical at-most-`3A` team allocation, outside-team claim
    protection, EPS tie order, suffix consensus, snapshot clearing, and
    prediction exclusion;
  - PI uses exact route marginal/significance/improvement priority,
    recomputes every locally owned significance after insertion, removes only
    invalid/lost tasks, and synchronizes the wrapper goal whenever consensus
    removes the old path head.
- Added matched computational metrics and Top-K provenance to simulator, hub,
  and onboard CSVs. Simulator and hardware now both record allocator and
  candidate-filter calls, totals, means, and maxima; simulator also records
  allocator median/p95. Hardware additionally records solve time excluding
  filter time, mean step time, and allocator percentage of trial time.
- Desktop acceptance evidence:
  - study simulator: 143 of 143 tests passed;
  - corrected `dcta_benchmark_sim`: 112 of 112 tests passed;
  - hardware: 139 of 139 tests passed;
  - ACBBA/CBAA/HIPC/PI stateful replay covers every K and EPS boundaries;
  - DGA covers every K at two seeds with exact 25-generation plans, fitness,
    and GA RNG state despite injected packet/backoff draws;
  - DMCHBA covers every K through initial solve, miss, later clue,
    peer-position change, retained path, exhaustion, and signature-triggered
    re-solve;
  - the full-K study simulator guard matches the corrected reference
    repository;
  - `mpy-cross` compiled the shared allocator module and all six Pololu
    programs;
  - the K=361 DMCHBA desktop probe completed with route
    `[(1,18),(0,17),(1,17)]` and 24,309 bytes of typed-array payload;
  - both scenario files are byte-identical and `git diff --check` passes.
- No desktop algorithm-parity gap remains. Physical acceptance is intentionally
  still open: use/flash binary64-capable RP2040 firmware, pass each robot's
  live K=361 heap/fragmentation and enlarged-UART-buffer probe, then run
  four-robot smoke trials for all algorithms at K=361, K=18, and at least one
  intermediate K. The smoke trials must exercise the full control barrier,
  start clues, collision/backoff, terminal freeze, repeated trials, and actual
  UART/MQTT delivery of protected intent and target traffic.

### 2026-07-25 — Manual memory-crash trial termination

- Added a nonblocking active-trial `M` key to `hardware/metrics_hub.py`. It
  does not require Enter. The hub freezes logical metrics at the keypress,
  exits the target wait, and sends the normal sequenced `ABORT` command so
  surviving robots stop even when another robot has crashed before sending a
  target alert.
- The existing post-trial memory-error prompt now classifies a manually ended
  trial explicitly:
  - yes writes `memory_error=1` and `trial_status=memory_error_crash`;
  - no writes `memory_error=0` and `trial_status=manual_stop`.
  Both the system and per-robot CSV rows retain the partial metrics and the
  operator/abort detail in `failure_reason`.
- The terminal reader supports immediate keys on Windows and POSIX terminals,
  restores POSIX terminal settings on every exit path, and retains `Ctrl+C` as
  the fallback for hosts without single-key terminal support. Automated runs
  do not enable the key reader.
- Added regressions for immediate key interruption, terminal-reader cleanup,
  confirmed memory-crash classification, and serialization of the flag,
  explicit status, and reason to both hub CSV outputs.
- Verification: the complete hardware suite passes 141 of 141 tests, and the
  modified Python sources compile successfully.

### 2026-07-25 — Self-contained Pololu allocator deployment

- Supersedes the earlier two-file deployment instructions in this log.
  `hardware/allocator_memory.py` was removed; each of the six active
  `Pololu_*.py` programs is now its complete device runtime and requires no
  companion Python module.
- The change addresses an observed immediate startup `MemoryError`. Loading
  the former helper created a second resident MicroPython module namespace
  before the programs allocated their 19-by-19 grids, binary64 probability
  maps, consensus tables, and Top-K workspaces.
- Embedded only what each program uses:
  - ACBBA, CBAA, HIPC, and PI contain the binary64 probe, fixed cell-indexed
    map, and packed Top-K candidate workspace;
  - DGA contains the binary64 probe and packed candidate workspace;
  - DMCHBA contains only the binary64 probe because its bounded Hungarian
    workspaces were already local.
- Every program runs the same strict binary64 round-trip check, deletes that
  one-shot function, and performs garbage collection before robot identity,
  grid, map, and allocator initialization. This removes the helper-module
  namespace without changing binary64 values, table operations, Top-K source
  order, overflow ranking, or algorithm equations.
- Desktop tests now extract the embedded primitives directly from the device
  scripts. Structural guards require no `allocator_memory` import, require
  identical shared primitive ASTs across applicable programs, reject unused
  primitive classes, and verify that the startup probe is run and released.
- Physical verification remains required: flash one self-contained algorithm
  file per robot and record successful boot plus live `gc.mem_free()` headroom
  at K=361, K=18, and an intermediate K. A persistent startup `MemoryError`
  after this change indicates that the algorithm's own global allocations,
  firmware heap, or fragmentation—not a second Python module—still exceeds
  the device budget.
- Desktop verification completed:
  - all 144 hardware and cross-implementation parity tests passed;
  - all 143 study-simulator tests passed, including the corrected-reference
    full-Top-K guard;
  - MicroPython v1.27 `mpy-cross` compiled all six self-contained programs;
  - compiled bytecode sizes were 32,844 bytes ACBBA, 28,801 CBAA, 36,871 DGA,
    26,683 DMCHBA, 29,996 HIPC, and 29,687 PI;
  - dependency scans found no Pololu or hardware-test import of
    `allocator_memory`;
  - Python compilation and `git diff --check` passed.

### 2026-07-25 — Temporary known-bootable hardware programs

- Physical startup still failed after the self-contained merge with
  `MemoryError: memory allocation failed, allocating 3112 bytes`.
- Read-only comparison against `dtca_benchmark_hardware` found that the
  Top-K/parity versions add roughly 9–16 KB of compiled bytecode plus
  substantial persistent binary64 tables/workspaces. The major increase came
  from the parity-alignment work, not from the later helper embedding.
- Moved the six current Top-K/parity programs to
  `hardware/in_the_works/`. They remain the target of desktop parity tests but
  are not approved for physical data collection until the memory issue is
  fixed and K=361 hardware gates pass.
- Copied the six known-bootable benchmark programs to `hardware/use/` for
  temporary physical startup, communication, motion, and hub testing.
- Kept `hardware/metrics_hub.py` unchanged. Each temporary robot program
  implements the current hub's topic-6 `CFGACK` and sequenced `CMDACK`
  protocol, applies the configured message-drop rate, supports protected
  abort/stop behavior, and uses starts `(0,0)`, `(0,6)`, `(0,12)`, and
  `(0,18)`.
- The temporary programs retain the benchmark allocator logic and do not
  implement Top-K filtering. They reject every configuration except
  `top_k_rate=1.0` / K=361. Their required `dcta_parity_v1` acknowledgment is
  transport compatibility for the unchanged hub, not evidence of allocator
  parity. Temporary host output uses a separate output directory/manifest
  lock, and robot-local files are named `metrics-temp-<ALGORITHM>.txt`.
  Temporary outputs must not be included in the Top-K study dataset.

### 2026-07-25 — Hub-configurable Top-K for temporary hardware programs

- Kept `hardware/metrics_hub.py` unchanged and extended only the six programs
  in `hardware/use/`.
- The temporary robots now accept the hub's six study rates (`1`, `.75`, `.5`,
  `.25`, `.10`, `.05`) and verify the corresponding rounded K values (361,
  271, 181, 90, 36, 18) before returning `CFGACK ... OK`.
- Every temporary allocator now consumes the configured K as its candidate
  limit. When truncation is needed, candidates are ranked by descending target
  probability, then local Manhattan distance, then cell coordinate. Algorithms
  whose benchmark source order is y-major preserve it when no truncation is
  needed; DGA and HIPC retain their existing probability-ranked ordering.
- Removed temporary DGA's hidden fixed 48-candidate cap so the configured and
  acknowledged K cannot differ from the allocator input.
- These files still use the older benchmark allocator equations and messaging
  behavior. Their Top-K support enables physical testing at different filter
  levels but does not make them simulator-parity study implementations.
- K=361 can create large runtime allocations in temporary DGA and DMCHBA.
  Physical smoke testing should start at K=18 and increase through an
  intermediate rate before K=361.
- Verification completed:
  - all 151 hardware tests passed, including hub ACK round trips for all six
    rates and controlled candidate-selection checks for every allocator;
  - all six temporary programs compiled with MicroPython `mpy-cross`;
  - `git diff --check` passed and `hardware/metrics_hub.py` remained unchanged.

### 2026-07-25 — Approved Pololu numeric-precision deviation

- The Pololu 3pi+ 2040's installed MicroPython firmware uses its compatible
  native scalar floating-point precision rather than the simulator's binary64
  scalar arithmetic. This hardware-versus-simulator precision difference is
  an accepted study deviation and must be disclosed with physical results.
- Hardware programs may use the Pololu-compatible numeric implementation and
  must not be prevented from starting solely by the strict binary64 startup
  probe. No runtime source change or firmware change was made as part of this
  decision entry.
- The large self-contained ACBBA source still exceeds available heap while
  MicroPython compiles it at startup. Deploying the same program as
  firmware-compatible precompiled `.mpy` bytecode is the preferred way to
  avoid that source-compilation peak without intentionally changing the
  ACBBA algorithm.
- The `.mpy` must be built for the robot's exact MicroPython bytecode ABI, with
  optimization disabled, and its behavior and deployment hash must be tested
  and recorded before physical study trials.

### 2026-07-25 — Robot 01 precompiled in-the-works deployment

- Updated all six programs in `hardware/in_the_works/` to use
  `ROBOT_ID = "01"` and to warn, rather than terminate, when the Pololu uses
  native scalar float precision. No allocator equation or task-selection
  behavior was changed.
- Compiled ACBBA, CBAA, DGA, DMCHBA, HIPC, and PI with `mpy-cross` in
  MicroPython 1.24 compatibility mode at optimization level zero. The six
  device `.mpy` files were verified byte-for-byte against their local builds.
- Removed same-name device `.py` files so they cannot shadow the compiled
  modules and recreate the source-compilation `MemoryError`. Added six
  `Run_*.py` menu launchers that only import their corresponding compiled
  modules.
- Changed the device's `main.py` default program to `None`; power-up now opens
  the Pololu selection menu instead of immediately calibrating and moving.
- Motor-free threaded startup reached `wait_for_trial_start` for ACBBA, CBAA,
  DGA, and DMCHBA. Complete compiled global initialization also passed for
  HIPC and PI, reporting approximately 119 KB and 120 KB free heap
  respectively. Repeated host-driven full-thread resets of HIPC destabilized
  the Windows USB serial connection, so a normal single-launch hardware check
  remains advisable before trial use.
- No task-allocation or physical trial cycle was performed. All temporary
  startup-probe modules were removed from the robot after verification.

### 2026-07-25 — Robot 01 motor-free Top-K allocation estimate

- Ran one local post-clue allocation on the physical Pololu using a clue at
  `(9, 9)`, no peer messages, no movement thread, and native Pololu float
  precision. These are ballpark single-robot limits rather than complete
  multi-robot trial validation.
- ACBBA passed K=361 in approximately 5.25 seconds.
- CBAA passed K=361 in approximately 0.89 seconds.
- PI passed K=361 in approximately 4.08 seconds.
- HIPC passed K=361 in approximately 20.00 seconds. It did not fail in the
  study range, but its full-grid allocation is a substantial communication
  pause.
- DMCHBA passed K=36 in approximately 3.53 seconds. K=90, K=181, and K=361
  were still inside the Hungarian solver after 30 seconds and were
  interrupted. No `MemoryError` was observed, so its practical boundary is
  between K=36 and K=90 under a 30-second allocation budget.
- DGA failed at both K=18 and K=361 before completing allocation because this
  MicroPython firmware does not implement `int.bit_length()`, which the
  embedded CPython-compatible random generator calls. This is independent of
  Top-K and means the current DGA deployment cannot complete an allocation at
  any study K until that compatibility issue is fixed.
- Temporary Top-K probe modules were removed after testing, and the production
  `.mpy` deployments were not modified.

### 2026-07-25 19:11 PDT - DGA MicroPython RNG compatibility fix and physical gate

- Replaced the unavailable `int.bit_length()` call only in
  `hardware/in_the_works/Pololu_DGA.py` with a positive-integer right-shift
  loop. The loop computes the same bit count before the existing
  `getrandbits()` rejection sampler, so it consumes no additional random draws
  and does not change the simulator, seed, population size, 25 generations,
  crossover, mutation, elitism, or any other GA operator.
- A bound-focused desktop check matched `random.Random.randrange()` for bounds
  `1, 2, 3, 4, 7, 8, 9, 255, 256, 257, 361`, using six seeds including robot
  01's seed `1010` and the large seed `(1 << 70) + 123`. All 2,640 draws and
  the final RNG states were identical. Existing random, shuffle, sample, RNG
  isolation, mutation, population, and complete 25-generation parity checks
  passed; the full `hardware.test_dga_simulator_equivalence` module passed all
  9 tests in an isolated copy of the committed test suite.
- Compiled only DGA with MicroPython 1.24 compatibility and optimization level
  zero using `mpy-cross -c 1.24 -O0`. Robot 01's deployed
  `Pololu_DGA.mpy` is 36,977 bytes and matched the local build at SHA-256
  `496C2BF69D0088BC7E40DACA4169C9B082609F7740204E6F694CB3E67107E2C3`.
- A motor-free, single-robot post-clue allocation at K=18 completed all 25
  generations in 63.295 seconds, selected task `(7, 8)`, and raised no
  compatibility or memory exception. Free heap was 116,464 bytes before the
  allocation and 60,624 bytes immediately afterward.
- The same motor-free allocation at K=361 exceeded the 180-second watchdog
  while still preparing the initial population (`dga_generation == 0`).
  It raised no compatibility or memory exception. Free heap was 114,992 bytes
  before allocation and 115,280 bytes after interruption and garbage
  collection.
- The original startup incompatibility is fixed, but K=361 does not satisfy
  the full-study acceptance gate. DGA remains blocked from full-grid physical
  trials pending a separate decision about a performance optimization that
  may affect parity. No GA parameter reduction was made.
- Removed every temporary probe, build directory, and isolated test worktree.
  The robot was soft-rebooted to `main.py` with `default_program = None`, and
  the motors were left off.

### 2026-07-26 - Immediate target stop and between-trial home rollback

- Updated all six programs in `hardware/in_the_works/` so a bump-triggered
  target detection turns both motors off before target publication, metrics
  work, alerts, or other target processing. A received peer target alert now
  also stops in-progress physical motion immediately.
- After returning to `START_POS`, each robot first restores its configured
  east-facing heading and then reverses both wheels at the existing base speed
  of 650 for 0.3 seconds before stopping. This reuses the programs' existing
  approximate one-inch motor timing.
- These are hardware motion-control changes only. They do not change candidate
  filtering, allocation equations, task selection, random streams, or the
  simulator algorithms.
- Compiled robot-00 deployments with `mpy-cross -c 1.24 -O0` and verified all
  six uploaded `.mpy` files byte-for-byte on `POLOLU 00`. Per request, no
  algorithm or physical-motion trial was run after deployment.

### 2026-07-26 - Robot 02 filesystem repair and updated deployment

- Robot 02 reproduced robot 00's menu failure: the Windows-mounted files
  appeared normal, but MicroPython's `os.listdir()` raised `UnicodeError`.
  Reflashing MicroPython 1.24 did not replace the separate user filesystem, so
  the verified 15 MB Pololu FAT volume was reformatted and labeled
  `POLOLU 02`.
- Restored the factory files, safe selection-menu `main.py`, and six launchers.
  Removed all old algorithms and logs through the format, then deployed fresh
  ID-02 `.mpy` builds containing the immediate target stop and east-facing
  between-trial rollback behavior.
- All 70 restored configuration, factory, and production files matched their
  source hashes. After a robot-side remount, both string and byte directory
  listings returned 36 valid entries with no non-UTF-8 names.
- No allocation or physical-motion trial was run during this repair.

### 2026-07-26 - Automatic return-home temporarily disabled

- In all six current simulator-parity Pololu sources, commented out the only
  active calls to target-retreat recovery and `return_home()`. The underlying
  functions remain intact for later repair, but they are unreachable from the
  repeated trial loop.
- After a trial, the robot now stops, records its metrics, remains at its final
  physical pose, resets its logical pose to `START_POS` facing east, and waits
  for the next `RUN` command. The operator must physically return each robot to
  its configured starting position facing east before that command.
- Target stopping and the normal search movement functions were left enabled.
  No line following, turning, collision handling, calibration, or allocation
  behavior was changed.
- All six files passed desktop syntax and control-flow checks and compiled with
  `mpy-cross -c 1.24 -O0`. No files were uploaded to a robot for this change.

### 2026-07-26 - Manual five-scenario metrics hub

- Revised `hardware/metrics_hub.py` to load only source episodes `4`, `53`,
  `232`, `394`, and `473` from
  `simulator/scenarios/final_trial_500.csv`. They are exposed to the operator
  as study trial IDs `1` through `5`, respectively.
- Removed unattended operation and automatic scenario scheduling. The hub now
  requires a valid manual scenario selection, physical-placement
  confirmation, Top-K and drop-rate confirmation, and memory-error reporting
  for every trial. `--trials N` remains available for manually operated
  multi-trial Top-K sweeps; repeated study IDs and Enter-to-reuse are
  supported.
- Removed the `--auto`, `--memory-error-default`, and `--start-index`
  arguments. `--trials` now defaults to one and rejects non-positive values.
- Added `source_trial_id` alongside the renumbered `trial_id` in published
  task JSON, system and robot metrics, event logs, command logs,
  configuration/control acknowledgment logs, and imported onboard metrics.
- Added expected-clue, unexpected-clue, and location-warning audit fields.
  Unexpected clue reports and target-location mismatches produce operator
  warnings but do not change an otherwise completed trial to a failure.
  Discovering every expected clue is not required.
- Added and locked
  `results/hardware_handpicked_5_scenario_manifest.json`. The manifest records
  both ordered ID lists and uses cohort SHA-256
  `92ebcdc84dc259fc27fc6123bef9ca9f0488a874e84e405344e349aa2d07d393`.
  Robots continue to receive the unchanged CFG/CFGACK protocol and acknowledge
  this cohort hash.
- Updated `hardware/README.md` with the five-ID mapping and the manual,
  repeated-ID Top-K workflow. No Pololu program or simulator architecture was
  changed for this revision.
- Added `tests/test_metrics_hub_manual_scenarios.py` outside `hardware/`.
  All 10 focused tests passed, covering remapping and provenance, the fixed
  hash and manifest, manual selection/reuse, rejected unattended arguments,
  repeated multi-trial operation, task and CSV ID propagation, mismatch
  warnings, row counts, PRESTART/START/RUN boundaries, and metric replay.
  `hardware/metrics_hub.py` and the test module also passed Python bytecode
  compilation.

### Next robot connection - Validate metric timing accuracy

- On the next robot connection, physically validate the Pololu metric output
  against an independent host monotonic clock or stopwatch.
- Check at minimum `trial_time_ms`, `motor_time_ms`, `compute_time_ms`,
  allocator timing, and the exact start/freeze boundaries for `RUN`, local
  target bump, peer target alert, and `ABORT`.
- Include a case where a peer target message arrives during an allocation.
  Confirm and document whether that robot's `trial_time_ms` includes the
  remaining allocation time before it processes the buffered target alert.
- Do not treat the robot-local timing fields as physically validated until
  these checks pass on connected hardware.

## 2026-07-26 - Paper-ready Top-K experiment and results text

- Added `results/PAPER_READY_TOPK_EXPERIMENT_AND_RESULTS.txt` as a standalone,
  copy-and-pasteable description of the completed primary simulation,
  sensitivity suite, and 50-target experiment.
- Included the paired compute/mission tradeoff, confidence intervals,
  sensitivity trend checks, failure disclosures, supported claims,
  unsupported claims, limitations, compact Methods and Results paragraphs,
  and source-data provenance.
- Explicitly separated completed results from the unrun 3%, 1%, K = 1, and
  RP2040 experiment proposals.

### 2026-07-26 - Separate allocator and candidate-filter timing

- Added timing to all six active, known-bootable `hardware/Pololu_*.py`
  programs without changing their allocator bodies. Each onboard row now
  reports candidate-filter calls and total/mean/maximum microseconds,
  allocator-solve total/mean/maximum microseconds with filter work excluded,
  and end-to-end allocator calls, total/mean/maximum microseconds, and trial
  percentage.
- Extended the local clue/coverage simulator's
  `computational_performance.csv` with `allocator_solve_time_ms_*` columns.
  Existing `allocator_time_ms_*` remains end-to-end, while
  `candidate_filter_time_ms_*` remains the complete candidate operation.
- Added the same three-way measurement to the sibling collaborative
  known-visit simulator with per-robot columns in `robot_performance.csv` and
  team totals in `system_performance.csv`. This was a metrics-only change to
  the known-visit repository.
- Verified all six Pololu sources compile and retain byte-for-byte-equivalent
  allocator/candidate AST bodies inside the new wrappers. The clue simulator
  passed 147 tests, the known-visit simulator passed 17 tests, the sensitivity
  suite passed 4 tests, and a one-trial known-visit CBAA smoke run produced
  nonzero filter, solve-only, and end-to-end allocator timings.

## 2026-07-27 - Collaborative rerun and Pololu-authoritative HIL campaign

### Completed 100-scenario collaborative simulation

- Replaced the prior 50-scenario collaborative-visit results with 100
  scenarios per condition for all six algorithms and six standard Top-K
  levels (5%, 10%, 25%, 50%, 75%, and 100%): 36 conditions and 3,600 mission
  runs.
- The scenario cohort uses seed `20311176`. Trials 0-49 are byte-for-byte the
  prior seeded rows and trials 50-99 are continued draws from the same random
  generator. The 100-row scenario file is
  `results/sensitivity_suite/scenarios/known_targets_g19_t50_n100.csv` with
  SHA-256
  `7537b0408132e9d6eaeb5544f867c6647609c309f1aa471ac5b8022e39405309`.
- The runner used `floor(logical_processors * 0.75)` workers. The recorded
  machine exposed 22 logical processors, so the campaign rule selected 16
  workers.
- `results/sensitivity_suite/_cv100/progress.json` records the campaign as
  complete with 3,600 jobs. The installed canonical results are under
  `results/sensitivity_suite/raw/collaborative_known_target_visit/`
  `topk_sensitivity/multitarget_g19_r4_t50`.
- Collaborative `system_performance.csv` rows now include team maximums
  derived from robot-level rows:
  `allocator_time_ms_team_max`,
  `allocator_solve_time_ms_team_max`, and
  `candidate_filter_time_ms_team_max`. These accompany the existing team
  totals and make the system schema consistent with the Bayesian timing
  outputs.
- Timing semantics remain:
  end-to-end allocator time includes filtering; candidate-filter time is the
  nested subset; allocator-exclusive/solve time is allocator time minus
  nested filter time. The three values must not be summed.

### Separate replay subsystem and live authority boundary

- Added the top-level `allocator_replay/` subsystem and dedicated results
  paths without editing or importing the active `hardware/Pololu_*.py`
  programs. Those files were read-only references. The benchmark simulator
  repositories were not behaviorally modified for HIL.
- Preserved the earlier offline trace/replay implementation and added a
  separate live Pololu-authoritative path:

  ```text
  simulator reaches choose_goal
  -> current allocator/robot state is sent over USB
  -> Pololu runs and times choose_goal
  -> Pololu returns goal, allocation messages, and mutable state
  -> simulator applies that response and advances
  ```

- The desktop simulator does not run a shadow `choose_goal()` during the
  campaign. It continues to own motion, sensing, environment events,
  communication delivery, and allocator callbacks outside `choose_goal()`.
  A complete emulator mission verified that the returned hardware-interface
  goal changes the subsequent simulator path: Bayesian CBAA K=1 trial 0
  completed with 426 total team steps, maximum 134 steps for one robot, and
  1,549 allocator calls in about 27.6 host seconds.
- The device timer uses `ticks_us()` only around `choose_goal()`. Fixture
  encoding/decoding, USB transfer, journaling, and parity checks are outside
  the timed region. Each call returns:
  allocator, candidate-filter, and allocator-exclusive microseconds; filter
  invocation count; candidates before/after filtering; heap before/after;
  goal; outbound messages; mutable robot state; and mutable allocator state.
- USB results use bounded ASCII chunks with sequence/CRC validation. No full
  mission trace is retained in host memory.
- Host commands are `hil-prepare`, `hil-run`, `hil-status`, and `hil-report`.
  Execution is append-only and resumable. If the host stops mid-trial, that
  trial restarts from its historical scenario with a new generation ID; the
  partial generation remains auditable but is excluded from representative
  summaries.
- Reports regenerate raw-call, robot-trial, system-trial, and condition CSVs.
  They include sample sizes and total, mean, median, p95, and maximum values
  for allocator, filter, and allocator-exclusive time, plus filter/call-path
  counts, `total_team_steps`, `max_steps_any_robot`, completion state, device
  identity, and scenario/source hashes.
- A Windows commit-memory guard prevents new trials from launching before
  virtual-memory commitment reaches the configured unsafe threshold. This
  was added after the earlier offline hardware replay process ended while the
  host was under abnormal memory/commit pressure.

### HIL testing matrix and device scheduling

- Algorithms: ACBBA, CBAA, DGA, DMCHBA, HIPC, and PI.
- Bayesian levels: K=1, 1%, 3%, 5%, 10%, 25%, 50%, 75%, and 100%;
  25 historical trials per condition. This is 54 conditions and 1,350 mission
  runs.
- Collaborative levels: K=1, K=2, 5%, 10%, 25%, 50%, 75%, and 100%;
  10 historical trials per condition. This is 48 conditions and 480 mission
  runs.
- Total planned matrix: 102 conditions and 1,830 mission runs.
- Work is partitioned at the complete-condition level. One Pololu owns one
  mission x algorithm x Top-K condition at a time. All calls within a trial
  stay sequential and on the same Pololu. When a device finishes, it receives
  the predicted-longest remaining unassigned condition so the two device
  queues finish as close together as practical.
- A disconnected device's active condition pauses and remains pinned to its
  stable `machine.unique_id()`. Reassignment is explicit and is logged as a
  mixed-device condition.
- Reproducible allocator timeout, memory, or invalid-output failures are
  confirmed with up to three attempts. Only the affected condition stops.
  USB transport failures are retried without counting as timing attempts.

### Initial differences between the two connected Pololus

| Property | COM12 | COM13 |
|---|---:|---:|
| Stable device ID | `e4621cb30b2b352f` | `e4621cb30b43372f` |
| Initial MicroPython | 1.22.1 | 1.24.0 |
| Initial MPY ABI | 4614 | 4870 |
| CPU clock | 125 MHz | 125 MHz |
| Initial repeated CBAA K=1 allocator median | 224.250 ms | 241.573 ms |
| Initial repeated end-to-end USB-call median | 2.101 s | 2.489 s |

- Before firmware alignment, both boards returned the same goal `(0, 9)`,
  one message, the same state shape, and a 50-to-1 candidate reduction, but
  allocator medians differed by about 8% and end-to-end call medians differed
  by about 18%. This exceeded the 5% cross-device calibration rule, so the
  boards were not accepted as a combined campaign pair.

### COM12 firmware alignment and filesystem preservation

- Before changing COM12, copied and hashed its entire `POLOLU 03` filesystem:
  178 files, 19 subdirectories, and 1,690,348 bytes. The verified backup is
  `results/allocator_replay/device_backups/`
  `e4621cb30b2b352f_pre_v124_20260727T2300Z`.
- Installed the exact firmware used by COM13:
  `POLOLU_3PI_2040_ROBOT-20241025-v1.24.0.uf2`, SHA-256
  `4fc62cff903000079ea0cd462c1b6e5f3e2a8f1ad3608e81439a52a8b22a8364`.
- Restored all 178 original COM12 files, including its original `main.py`,
  algorithms, logs, hidden files, trash, and volume metadata. A final
  file-set, byte-count, and SHA-256 comparison had zero missing, extra, or
  mismatched original files.
- Deployed replay-only modules without changing `main.py`. Both devices now
  report MicroPython 1.24.0, MPY ABI 4870, 125 MHz, firmware SHA-256
  `c64e31b161421148abdd87a4eea541048456e8e99e6db08407499b0dfa7bfb7c`,
  replay build `micropython_1_24_o0_5fc60d72663c`, and module-set SHA-256
  `d7d86507c8b2a48cfde5a7e07172e8625a6ffccd8b86cd5bd6629934958756fd`.

### Post-alignment physical smoke and calibration metrics

| Metric | COM12 | COM13 |
|---|---:|---:|
| Five-call allocator median | 239.452 ms | 239.971 ms |
| Five-call filter median | 226.288 ms | 226.549 ms |
| Five-call end-to-end USB median | 2.038 s | 2.045 s |
| Preflight heap at identity check | 166,496 B | 166,896 B |
| Preflight free heap after module check | 135,808 B | 136,208 B |

- The post-alignment allocator-median spread was 0.22%. All ten calls
  completed and returned identical goal/message/candidate outputs.
- Joint calibration used two repetitions per reference fixture:
  Bayesian CBAA 5% medians were 3,096.5 us and 3,190.5 us (about 3.0% spread);
  collaborative CBAA 5% medians were 244,281.0 us and 244,894.5 us (about
  0.25% spread). Both are within the 5% rule.
- Joint preflight passed matching firmware, implementation, build, module
  hashes, and clock checks. Both boards passed safe REPL exit. Replay modules
  reported that motors and sensors were never initialized.
- One detailed authoritative collaborative CBAA K=1 call on each device
  reduced 50 candidates to 1, invoked the filter once, returned one allocator
  message and goal `(0, 9)`, and reported about 13-14 ms allocator-exclusive
  time. Most of that call's approximately 240 ms allocator time was the
  approximately 226 ms candidate filter.

### Observed resource limits and failure-adjusted duration

- Both boards reproduced a Bayesian DGA 5% state-load `MemoryError` while
  allocating 1,024 bytes. Additional probes reproduced the same failure at 1%
  and 3%. Bayesian DGA K=1 remains unclassified because its archived smoke
  trace is incomplete.
- Both boards exceeded 30 seconds on the collaborative DGA 5% smoke fixture.
  Additional K=1 and K=2 probes also exceeded 30 seconds. Because increasing K
  cannot reduce this DGA workload, all eight collaborative DGA conditions are
  expected to stop early.
- The earlier offline physical replay had already classified Bayesian DGA 75%
  and 100%, and Bayesian DMCHBA 75% and 100%, as reproducible
  `memory_unusable` conditions. Bayesian DGA 50% was active with zero completed
  fixtures when that host process ended.
- Current planning therefore expects at least 18 early-stopped conditions:
  all eight collaborative DGA levels; Bayesian DGA 1% through 100% (eight
  levels); and Bayesian DMCHBA 75%/100%. Bayesian DGA K=1 is retained
  conservatively until the live campaign classifies it.
- After excluding those expected early stops, the clue-qualified cohort is
  estimated at 84 conditions, 1,500 completed mission runs, and about 249,763
  hardware allocator calls. At the observed approximately 2.04 seconds per
  representative USB round trip, the two-device theoretical floor is about
  2.95 continuous days. The practical planning range is 4-6 days, with seven
  days reserved for larger states, higher-K compute, retries, and reconnects.
  Pololu-authoritative decisions can change mission paths and therefore the
  final call count.

### Clue-qualified Bayesian cohort

- Audited all 500 historical Bayesian trial IDs across the 36 available
  standard Top-K algorithm conditions. A clue was recorded in every condition
  for 495 trials.
- Clue discovery alone was insufficient for the intended allocator benchmark:
  four trials in the first sample (`26`, `36`, `448`, and `497`) recorded a
  clue but zero post-clue allocator calls. Eligibility was therefore defined
  as both:
  1. clue found in every available historical condition; and
  2. at least three post-clue allocator calls in every such condition.
- There are 379 eligible trials. Applying the unchanged Bayesian sampling
  seed `20260727` selected:
  `7, 12, 32, 45, 50, 56, 67, 80, 94, 108, 127, 132, 164, 166, 178, 216,`
  `226, 233, 274, 313, 358, 371, 388, 398, 424`.
  Every selected trial has at least four post-clue calls in its weakest
  historical condition.
- Collaborative selection remains a ten-trial fixed-seed sample from trials
  0-99 using seed `20311176`.
- The executable immutable campaign is
  `results/allocator_replay/hil_campaigns/`
  `pololu_authoritative_clue_qualified`. The older
  `pololu_authoritative_initial` manifest is retained only as a superseded
  audit record and must not be executed.
- The final allocator replay/HIL suite passed all 17 desktop tests, covering
  authoritative state and message round trips, hardware authority over the
  simulator path, chunking, timing fields, timeout/memory handling,
  one/two/three-device scheduling, disconnect/resume, report regeneration,
  deterministic clue-qualified selection, and rejection of an insufficient
  eligible population.
- No HIL mission trial has started. The clue-qualified schedule remains
  `prepared` with 102 pending conditions and 0 of 1,830 planned runs complete.

### Campaign launch and pre-timing failure handling

- Started the immutable `pololu_authoritative_clue_qualified` campaign on
  COM12 and COM13 on 2026-07-27. The host runner is a hidden, resumable
  process with append-only per-device journals under the campaign directory.
- The first DGA/100% call on COM12 and a later DMCHBA/100% call on COM13
  returned structured device failures before a `TIMED` packet could be
  emitted. The host initially mislabeled these as USB `transport_error`
  events and paused both workers even though Windows still saw both serial
  ports.
- Updated only the replay host transport (not any original Pololu program or
  benchmark simulator) so a chunked authoritative failure that precedes the
  timed boundary is consumed and classified by its real device failure type.
  Added and passed a loopback regression test for this exact protocol path.
  The original generation-1 transport records remain in the journals for
  auditability and are excluded from completed-run summaries.
- Resumed the same schedule. COM12 then reproduced DGA/100% memory failure
  three times, stopped only that condition as `memory_unusable`, reproduced
  DGA/75% memory failure three times, stopped that condition, and was
  automatically assigned Bayesian DMCHBA/75%. COM13 continued Bayesian
  DMCHBA/100%. This verified condition-level failure isolation and dynamic
  next-condition scheduling on both physical boards.

### HIL campaign paused: feasibility classifications are not valid

- Paused the Pololu-authoritative campaign after the high-K failure pattern
  contradicted known native Pololu behavior. Stopped the hidden host process,
  returned both interrupted conditions to `pending`, marked both devices and
  the central schedule `paused`, and confirmed that COM12 and COM13 remained
  discoverable. No campaign runs were resumed.
- At pause time the schedule had zero completed trials, 75 pending
  conditions, and 27 stopped conditions: 22 labeled `memory_unusable`, four
  labeled `timing_unusable_30s`, and collaborative DGA 75% labeled
  `hardware_call_failed`. These labels must be treated as preliminary HIL
  failures, not native allocator feasibility results.
- The first Bayesian CBAA call state was 36,762 serialized bytes. Its
  361-cell `target_p` map was serialized twice, once under the robot views and
  once under the belief object, at 17,310 bytes per copy. Together those
  redundant copies were about 94% of the fixture. Native CBAA instead keeps
  compact persistent `array('f')` probability maps and never reconstructs
  them from USB on every allocator call.
- All three CBAA 50%, 75%, and 100% campaign attempts failed before
  `choose_goal()` at the same 4,089-byte `PEND` JSON decode. Thus none of
  those failures measured CBAA allocator memory.
- Ran two isolated motor-free COM12 diagnostics without changing deployed
  files. Removing the redundant belief copy and reducing transfer batches
  allowed the same Bayesian CBAA 100% state to execute:
  - the first pre-clue call completed in 2,883 us;
  - the first post-clue full allocation completed in 1,843,635 us, including
    552,178 us of candidate filtering and 1,291,457 us allocator-exclusive;
  - desktop and Pololu both selected `(2, 11)`, both reported 355 candidates
    before and after the 100% filter, and the Pololu retained 32,528 bytes of
    heap after the call.
  This proves CBAA 100% is feasible for that actual simulator allocator call
  and that the campaign's CBAA memory classifications were replay packaging
  artifacts.
- The current HIL protocol creates and restores a new allocator and
  `ReplayRobot` from a complete snapshot for every call, duplicates immutable
  state, and builds a complete JSON post-state in memory before chunking it.
  Native Pololu programs keep one robot's state resident and exchange only
  incremental peer/allocation messages. The HIL output path also produced
  memory failures after 18 calls had already completed `choose_goal()`;
  those cannot be interpreted as allocator failures.
- The host's 30-second deadline begins after the device sends `START`, but
  `START` is sent before allocator construction, robot reconstruction, state
  restoration, and garbage collection. The recorded microsecond metric times
  only `choose_goal()`, but the feasibility timeout currently covers more
  than `choose_goal()`. Existing timing classifications therefore also
  require retesting with a corrected boundary.
- Collaborative DGA 75% failed three times on
  `route[start:end] = reversed(route[start:end])`. CPython accepts the
  iterator for slice assignment, while MicroPython 1.24 requires a tuple or
  list. The generated port copied this expression unchanged, and the current
  build transform does not rewrite it. Native `Pololu_DGA.py` uses a
  different in-place reversal and does not contain this expression. The
  `hardware_call_failed` row is a replay-port compatibility defect, not a DGA
  feasibility result.
- The current `hardware/Pololu_*.py` programs and HIL allocator ports are not
  equivalent execution targets. The hardware README explicitly labels the
  native programs as temporary known-bootable older benchmark logic rather
  than simulator-parity implementations. For example, native DGA uses a
  population of 12 and eight iterations per trigger, while both simulator
  DGA implementations used by HIL use a population of 30 and 25 iterations.
  Native probability maps use float32 arrays; the simulator-parity replay uses
  binary64 values. Native success therefore does not establish that every
  full simulator allocator fits unchanged, although the corrected CBAA probe
  shows that HIL overhead—not CBAA—caused the observed CBAA failures.

## 2026-07-27 corrected native persistent Pololu HIL implementation

### Prior campaign invalidation

- The paused `pololu_authoritative_clue_qualified` campaign is diagnostic
  evidence only. It now contains `INVALID_FOR_ANALYSIS.json`; its append-only
  journals are preserved, but regenerated reports accept zero attempts,
  trials, or conditions as representative hardware results.
- The earlier `pololu_authoritative_initial` manifest is also superseded.
  Neither campaign may be resumed for analysis. The false memory/timing labels
  described above came from full-snapshot setup, output packaging, and a
  misplaced timeout boundary rather than the native allocator call.

### Native code that is now timed

- Added a separate implementation under `allocator_replay`; no
  `hardware/Pololu_*.py`, archived allocator, benchmark simulator, or device
  `main.py` file was edited.
- Bayesian HIL and the stationary/moving physical adapter now instantiate the
  same complete MicroPython allocator factory. CBAA uses normalized
  probability scoring, ACBBA uses its native ACBBA scoring, and DMCHBA uses
  the current scoring. Bayesian DGA retains the study search size of 30 plans
  and 25 generations and the same five mutation strategies. Memory-oriented
  packed plans change representation, not the search operations or scoring.
- Collaborative visit now has a practical, motor-free native 50-target
  implementation for CBAA, ACBBA, PI, HIPC, DMCHBA, and DGA, organized under
  `allocator_replay/device/native/collaborative`. It follows the same
  MicroPython design style as a deployable Pololu allocator and uses the same
  complete factory for HIL and a future physical wrapper.
- Floating-point representation was not forced to match the desktop
  simulator. The benchmark target is instead consistent native execution:
  the HIL call and a later real-testbed call use the same deployed allocator
  modules and strategies.

### Corrected persistent call boundary

- The simulator sends trial configuration once and maintains one resident
  robot/allocator context per Pololu. When the simulated scheduler switches
  robots, compact allocator-owned state is restored outside timing; immutable
  environment state is not echoed in every result.
- `PSETUP` completes and the board returns `PCALL_READY` before the host starts
  the 30-second allocator deadline. `PTIME` then measures only
  `choose_goal()` with `ticks_us()`. The board emits `PTIMED` before messages,
  state serialization, journaling, or USB transfer.
- Every call records total allocator time, cumulative nested candidate-filter
  time, allocator-exclusive time, filter invocation count, candidates before
  and after filtering, call-path classification, and free heap before and
  after the timed call.
- Setup, timed allocator, output serialization, and USB transport failures are
  journaled as separate phases. Only reproducible failures in the timed
  allocator classify a condition as `memory_unusable` or
  `timing_unusable_30s`; packaging failures can no longer masquerade as
  allocator failures.
- The Pololu's returned goal, allocation messages, and mutable allocator state
  drive the next simulator step. The desktop allocator does not shadow or
  override `choose_goal()`.

### MicroPython compatibility and bounded state transfer

- Rewrote generated DGA iterator slice assignment and generator-to-array
  extension into MicroPython-compatible list/append operations. Also removed
  CPython-only tuple `startswith` and class `__mro__` assumptions.
- Bayesian DGA initially completed its K=1 timed solve but failed afterward
  while copying its RNG state. The exact 2,500-byte request was the
  625-pointer temporary created by `Random.getstate()`. The output worker now
  streams 24 RNG words per bounded byte field without calling `getstate()`,
  then reconstructs the ordinary flat MT19937 state before the next native
  call.
- DGA populations are sent one plan at a time. Packed plan cells are also
  component-split into bounded chunks so higher Top-K state never requires one
  large post-call JSON allocation. Context-switch tests prove that the next
  decision and resulting state match an uninterrupted resident context.

### Final verification and immutable campaign

- The complete desktop suite passes 68 of 68 tests. Coverage includes all 12
  native mission/algorithm engines using captured historical states,
  authoritative path changes, RNG and packed-plan context restoration, timing
  boundaries, output chunking, invalid goals/state, timeout confirmation,
  one/two/three-device scheduling, disconnect/resume, journal deduplication,
  report regeneration, and source/scenario provenance.
- Both boards passed physical preflight on replay build
  `micropython_1_24_o0_b95d70ed28a7`, deployed module-set SHA-256
  `b719825eb3f600714d172d91cb56e43cbc49cbf084e3848d5a846994e7384461`.
  Firmware, module hashes, 125 MHz clock, and implementation match. All 24
  restore/delta smokes completed, deterministic K=1 parity passed for both
  missions, motors and sensors remained uninitialized, and both devices
  safely exited to and restarted from REPL.
- Five repeated persistent CBAA calibration calls per board differed by at
  most 0.83%, within the 5% combination rule. Bayesian DGA K=1 completed its
  full solve and state return on both boards in about 20.0 seconds; this
  indicates that larger DGA Top-K settings may legitimately reach the
  30-second timed cutoff.
- The immutable schema-v2 campaign is
  `results/allocator_replay/hil_campaigns/pololu_native_persistent_v2`.
  Manifest SHA-256 is
  `dd32b7fc67bb05dbbb211dd1337748fd29c1a1018eed9ccd3905f762874b7d09`.
  It binds the exact device build, all allocator/host/simulator sources, and
  every historical scenario hash; execution fails closed if any source,
  scenario, or connected build drifts.
- Bayesian uses the clue-qualified trial IDs
  `7, 12, 32, 45, 50, 56, 67, 80, 94, 108, 127, 132, 164, 166, 178, 216,`
  `226, 233, 274, 313, 358, 371, 388, 398, 424` for every algorithm and
  K=1/1%/3%/5%/10%/25%/50%/75%/100%.
- Collaborative visit uses historical trial IDs
  `10, 12, 14, 26, 30, 31, 32, 41, 49, 74` for every algorithm and
  K=1/K=2/5%/10%/25%/50%/75%/100%.
- The matrix contains 102 complete condition jobs and 1,830 mission runs.
  Conditions are assigned dynamically to COM12 (`e4621cb30b2b352f`) and
  COM13 (`e4621cb30b43372f`), one entire condition at a time per device,
  longest predicted work first. Trials and calls remain sequential and pinned
  to one board; journals make interruption and restart idempotent.

### Clean-VM correction and final campaign generation

- The preceding `pololu_native_persistent_v2` campaign is invalid for
  analysis. Second-robot restores failed during `PDATA`/`PEND`, before
  `PCALL_READY`; all 28 raw attempts are retained as diagnostics and zero are
  accepted. `PCLEAR` now releases the prior runtime before a restore, logical
  payload pieces are capped at 768 bytes, and raw-array pieces at 384 bytes.
- `pololu_native_persistent_v3` verified that fix with zero setup failures,
  but is also invalid for analysis. DGA timed calls completed and then failed
  a 376-byte output allocation. The same MicroPython VM had previously
  imported all 12 allocators during preflight and retained only about 70 KB
  free. Its 16 raw attempts are diagnostic and zero are accepted.
- Every condition now begins with a raw-REPL soft reset, and every retry after
  a failed attempt receives an independent soft reset. This bypasses
  `main.py`, imports only `replay_worker`, and prevents allocator-module code,
  interned strings, fragmentation, or state from carrying into another
  condition. Both COM12 and COM13 independently reported exactly 165,952 bytes
  free at the fresh worker identity boundary; motors and sensors remained
  uninitialized.
- A clean COM12 diagnostic replayed the exact Bayesian trial-7 DGA K=361
  call that had failed under the contaminated VM. It completed in 246,632 us,
  its timed-call heap changed from 113,968 to 58,944 bytes, and its complete
  output returned, including 56 robot-state fields. This was a high-K
  historical call, not a claim that every later post-clue DGA solve will meet
  the 30-second limit.
- The worker now garbage-collects only after `PTIMED`, before output
  serialization; the recorded heap-after value remains the value at the end
  of the timed allocator call. Host output errors record the output stage and
  field index. Discovery now explicitly exits a raw prompt before evaluating
  its identity query, so either worker/REPL state is recoverable.
- The final desktop verification suite passes 69 of 69 tests.
- The final replay build is `micropython_1_24_o0_3ac0f4d66ab4`, with deployed
  module-set SHA-256
  `e77946bdfab97eeba571ef9b172891def5bf73764eee452407bffaa162627669`.
  Both boards passed the final physical preflight: 24 of 24 native
  restore/delta smokes completed, all four deterministic mission parity
  checks passed, firmware/build/module hashes and 125 MHz clocks matched,
  safe exit/restart passed, motors and sensors remained uninitialized, and
  the maximum cross-device calibration deviation was 0.752% against the 5%
  limit.
- The only campaign intended for analysis after these corrections is
  `pololu_native_persistent_v4`. Its immutable manifest hash and launch
  process are recorded below after preparation.
- Prepared `pololu_native_persistent_v4` with campaign-manifest SHA-256
  `2ad6f680d8c23a200060abb1e927011841f569f73672232d4b93600e66f2b56b`.
  All ten fail-closed provenance checks passed against both connected build
  IDs. The sealed matrix contains 54 Bayesian conditions/1,350 missions and
  48 collaborative conditions/480 missions, for 102 conditions and 1,830
  missions total. The Bayesian sample remains the fixed 25-trial sample from
  379 clue-qualified historical trials; the collaborative sample remains the
  fixed ten historical trials listed above.

## 2026-07-27 final RNG, DMCHBA, and timeout-recovery corrections

### DGA context restoration and v4 invalidation

- `pololu_native_persistent_v4` is invalid for analysis. The first
  multi-robot DGA context restore tried to construct an uninitialized
  `Random` object through `Random.__new__`, an operation unavailable on
  MicroPython 1.24. This was a context-restore artifact outside the timed
  allocator, not a DGA allocator result. Its raw records remain preserved and
  none are accepted for analysis.
- DGA now restores the exact MT19937 continuation through the normal supported
  constructor, `Random(None, restored_state, index)`. The receiver
  preallocates one `array('I')` directly from a `bytearray(length * 4)` and
  streams the 624 state words into it. It does not create a 624- or
  625-element Python pointer list and does not reseed or approximate the RNG
  state.

### Complete native Bayesian DMCHBA

- The earlier Bayesian DMCHBA HIL factory still selected the generated
  simulator port. That representation retained a 361-entry tuple-key
  probability dictionary, built the candidate list twice, stored a large
  tuple signature, and allocated overlapping assignment workspaces. Its
  high-K memory behavior was therefore not a defensible native DMCHBA
  feasibility measurement.
- Bayesian DMCHBA now uses one complete, self-contained native MicroPython
  implementation for both HIL and the future physical adapter. It keeps flat
  float32 probabilities, packed uint16 candidate IDs and assignment
  signatures, computes assignment costs on demand, and uses typed,
  preallocated Hungarian vectors rather than an agent-by-task cost matrix.
  Its scratch storage grows in O(N); the full 361-cell/four-robot workspace
  payload is 16,375 bytes.

### Final build and verification

- The final replay build is `micropython_1_24_o0_71ffde22820c`.
  Its source-bundle SHA-256 is
  `71ffde22820c75d218c7e774ee2c6c45814568287a2cde5b0451eac221eb1dd5`
  and its deployed module-set SHA-256 is
  `39ca177708ca0120f47caf050627fa7c88a1e863668e8f58fbd0ece12e5365d0`.
- The complete desktop verification suite passes 89 of 89 tests.
- Both physical boards passed the final preflight: 24 of 24 native
  restore/delta smokes, four of four deterministic parity checks, and two of
  two forced DGA context restores passed. Firmware, build, module hashes, and
  125 MHz clocks matched; maximum cross-device timing deviation was 0.34%.
  Motor and sensor initialization counts remained zero on both boards.

### v5 timeout-recovery finding and correction

- `pololu_native_integration_canary_v5` ran historical Bayesian trial 7 at
  DGA 100% and native DMCHBA 100%. Each condition completed the same six
  pre-clue allocator calls, then robot 01 call 1 acknowledged
  `PCALL_READY` and exceeded the 30-second timed boundary once.
- Those single attempts were real observed deadline exceedances, but not
  confirmed `timing_unusable_30s` classifications. After Ctrl-C, the replay
  worker emitted `INTERRUPTED` and deliberately continued. The old restart
  path discarded that acknowledgement, expected raw REPL immediately, and
  marked both healthy USB devices disconnected before repetitions 2 and 3.
  The v5 canary is invalidated for this timeout-recovery artifact, with zero
  representative samples.
- Timeout recovery now acknowledges `INTERRUPTED`, asks the idle worker to
  `EXIT`, observes `BYE`, returns to raw REPL, performs a soft reset, and
  verifies the same device identity and build before restoring the identical
  fixture for the next repetition. Recovery success or transport failure is
  journaled as its own phase. Timeout rows also retain the heap reported at
  `PCALL_READY`, measured host elapsed time, and the configured threshold.
  The two-of-three rule is applied only after independent clean-VM attempts;
  recovery or USB failure never counts as an allocator timeout.

### v6 recovery canary and prepared analysis campaign

- `pololu_native_integration_canary_v6` passed the targeted recovery gate. For
  each of Bayesian DGA 100% and native Bayesian DMCHBA 100%, it retained six
  completed pre-clue calls; repetitions 1, 2, and 3 of the first post-clue
  call each exceeded 30 seconds; all three clean-worker recoveries completed;
  and there were zero transport errors. Each affected condition was correctly
  isolated as `timing_unusable_30s`.
- That canary is sealed `integration_canary_only`. Its append-only raw
  evidence remains available, but no canary attempt, trial, or condition is
  accepted as representative analysis data.
- Prepared the immutable `pololu_native_persistent_v6` manifest with SHA-256
  `3df4ac7cfb23e43a0b51110f3ad479e1fc6cff0b1611be9e151196cfa61b838e`.
  It contains the full 102-condition, 1,830-mission matrix and binds the final
  build, allocator and host sources, simulator sources, historical scenarios,
  and trial-selection evidence. This entry records preparation only; it does
  not claim that the full campaign has been launched.

### Full v6 campaign launch

- Launched `pololu_native_persistent_v6` at 2026-07-27 21:01:37 PDT as hidden
  host process PID 18304, using both automatically discovered Pololus.
  Append-only journals, process output, and the PID file are under
  `results/allocator_replay/hil_campaigns/pololu_native_persistent_v6/`.
- The longest predicted conditions were assigned first. COM12 initially owned
  Bayesian DGA 100% and COM13 initially owned native Bayesian DMCHBA 100%.
  Each completed six normal pre-clue calls, then independently produced three
  acknowledged 30-second timeouts on the first post-clue call. All recovery
  cycles completed and no transport error occurred. The scheduler correctly
  stopped only those two conditions as `timing_unusable_30s`.
- After those classifications, both boards remained connected and the same
  host process continued scheduling work: COM12 moved to Bayesian DMCHBA 75%
  and COM13 moved to Bayesian DGA 75%. At this checkpoint the campaign had two
  stopped conditions, two running conditions, and 98 pending conditions.
- Both 75% conditions subsequently completed the same three-attempt timeout
  confirmation without transport errors and stopped in isolation. The
  scheduler then moved COM12 to Bayesian HIPC 100% and COM13 to Bayesian ACBBA
  100%. At the unattended handoff checkpoint, PID 18304 remained healthy with
  four stopped conditions, two running conditions, and 96 pending conditions.

## 2026-07-28 v6 pause, state-transfer corrections, and v7 continuation

### Safe pause and v6 evidence audit

- Stopped the exact hidden v6 runner PID 18304 after verifying its command
  line, soft-reset both Pololus to clean MicroPython sessions, and confirmed
  COM12 and COM13 remained discoverable. The two interrupted conditions were
  returned to pending; incomplete ACBBA 50% trial 12 generation 1 and HIPC
  50% trial 14 generation 1 remain in append-only journals but are excluded
  from completed summaries.
- At the pause checkpoint, v6 contained 97 completed trials, nine fully
  completed conditions, 33 stopped conditions, and 60 pending conditions.
  Seventeen stopped conditions had defensible `timing_unusable_30s`
  classifications: exactly 51 acknowledged timeout attempts, repetitions
  1-3 for every condition, 51 completed clean-worker recoveries, and zero
  transport errors.
- Audited all completed collaborative data: 93 completed trial generations
  and 20,203 accepted allocator calls. There were zero setup, decoding,
  memory, transport, invalid-output, or phase failures in those completed
  trials; no duplicate generations or call-index gaps; all scenario hashes,
  build IDs, device ownership, and 125 MHz clocks matched. Every timing row
  satisfied `allocator_time_us = candidate_filter_time_us +
  allocator_exclusive_time_us` exactly. No completed collaborative condition
  requires rerunning.

### Bayesian typed-array root cause and correction

- All 15 `ValueError: bytes length not a multiple of item size` failures
  occurred immediately after the first successful post-clue output from
  generated Bayesian CBAA, ACBBA, PI, or HIPC. MicroPython 1.24 arrays expose
  neither `.typecode` nor `.itemsize`; the old serializer therefore defaulted
  every array to float32. A 361-element candidate `array('H')` occupied 722
  bytes but was labeled float32, reproducing the exact host exception.
- The controller no longer guesses a type for a type-opaque MicroPython array.
  Bounded logical arrays fall back to item encoding. The five reusable
  candidate buffers are preallocated during the shared HIL/physical setup
  boundary and excluded from logical snapshots because they are transient
  implementation workspaces. HIL and the new physical adapter both perform
  this preparation before starting `ticks_us()`, so the timed
  `choose_goal()` boundary remains identical.

### Bayesian DMCHBA setup-memory root cause and correction

- The Bayesian simulator exposes one `Belief.searched` set through
  `views.searched`, `views.local_searched`, and `belief.searched`. The old HIL
  transfer decoded three independent copies. Late in a mission, each encoded
  copy was about 8.5 KB plus tuple/set heap, explaining the repeated
  `PDATA`/`PEND` setup `MemoryError` events in DMCHBA 10%.
- Large sets now stream in parts no larger than 768 encoded bytes. Explicit
  state aliases restore the three searched paths to one object. After native
  DMCHBA consumes that logical set during untimed setup, it retains a compact
  361-byte searched bitmap, matching the native physical grid footprint.
- Offline authoritative Bayesian trial 50, which previously stopped during
  setup, now completed all 241 allocator calls, 412 total team steps, and 105
  maximum steps by one robot. All 25 DMCHBA 10% trials are intentionally
  rerun under the corrected build so that condition does not mix heap
  footprints or build revisions.

### Corrected build and physical gates

- The complete allocator-replay suite passes 92 of 92 tests and compilation
  checks. The corrected device build is
  `micropython_1_24_o0_5af608a14777`, with source-bundle SHA-256
  `5af608a14777d6fc15c8f7f034d662cfd36f2c0099895c870e4bbae4382d25d0`,
  deployed module-set SHA-256
  `e9502f20c40188d663877c540725132f4c047ca02a113109ae3cfc180c784599`,
  and build-manifest SHA-256
  `86dd81061c8bff12ae15d0108f1e797b92a191e800f6ff8ccdc6aa794e86074a`.
  Both boards received the build with `main.py` unchanged.
- The new two-board physical preflight passed: 24 of 24 persistent smokes,
  four of four parity checks, two of two forced DGA context restores, matching
  firmware/build/module hashes and 125 MHz clocks, safe REPL exit, and zero
  motor or sensor initialization. Five-repetition cross-device calibration
  differed by only 0.0104%.
- A dedicated 19x19 Bayesian DMCHBA K=36 setup gate used 360 searched cells
  and ran restore, resident delta, and forced restore on both boards. All six
  calls returned the expected goals with no `PDATA`, `PEND`, or setup error.
  Heap before timing remained between 99,760 and 111,408 bytes for the first
  two phases and above 110,700 bytes after forced restore. The saved gate JSON
  has SHA-256
  `44a25a011a6f08cd05fb36ca9d9e1d122da5930253b4bd46fc490c2488a3d5a8`.
- Two diagnostic-only actual-trial canaries crossed the former post-clue
  decoder boundary repeatedly. ACBBA/HIPC produced 46 completed hardware
  calls and CBAA/PI produced 22, with zero condition or transport errors.
  Both canaries were intentionally stopped after satisfying the gate and
  sealed `integration_canary_only`; none of their rows is analysis data.

### Provenance-preserving v7 launch

- V6 is frozen as a superseded partial campaign. `CONTINUED_BY.json` retains
  the exact valid/invalid scope. Its 93 audited collaborative trials and 17
  confirmed timing classifications remain under the original v6 manifest and
  build; corrected source must never be resumed into v6.
- Prepared `pololu_native_persistent_v7` with immutable campaign-manifest
  SHA-256
  `5510c684186c0a7f205d0713a426185f20081cb8368c8403a7b069d5e2f84d8a`.
  Its schedule contains the 76 invalid or unfinished conditions and 1,387
  mission runs: all 16 invalid conditions were requeued, DMCHBA 10% was reset
  to all 25 trials, and the three audited completed trials within the two
  interrupted collaborative conditions were omitted. `CARRY_FORWARD.json`
  defines how final reporting joins v6 and v7 without rewriting either.
- All ten provenance checks passed against both connected boards. Launched
  v7 as hidden PID 25032 at 2026-07-28 07:50 PDT. COM12 began retesting
  Bayesian ACBBA 100% and COM13 Bayesian PI 100%, two conditions previously
  stopped by the host decoder. At the monitored handoff, both had crossed the
  former failure point with at least nine completed calls each and zero setup,
  decoding, timeout, transport, or condition errors.
- These corrections changed only `allocator_replay` and its versioned result
  metadata. No legacy `hardware/Pololu_*.py`, device `main.py`, archived
  allocator, or benchmark simulator source was edited.

### Early v7 memory-classification qualification

- Bayesian ACBBA 100% crossed the former typed-array decoder failure, producing
  19 completed calls. Robot 01 call 2 then produced one setup-memory failure
  followed by two reproducible failures inside timed `choose_goal()`. The two
  confirmations each began with 31,440 bytes free, failed after about 715.5 ms
  while requesting 2,048 bytes, and recovered cleanly. The scheduler therefore
  correctly stopped this HIL condition as `memory_unusable` under its
  two-of-three rule.
- This is presently classified more narrowly as
  `hil_fixture_memory_unusable`, not proof that resident native ACBBA 100% is
  unusable on a real testbed. HIL multiplexes four simulator robots through
  one controller and reconstructs a complete robot context when ownership
  switches; the physical adapter keeps one robot context resident and applies
  compact deltas. Other ACBBA 100% calls with similar candidate counts
  completed when they began with more heap.
- The campaign remains running because the evidence does not indicate a
  global allocator-correctness failure. The stopped row is quarantined from
  physical-testbed feasibility claims in `CLASSIFICATION_NOTES.json` and
  requires a resident-context hardware probe before final interpretation.
  A later ACBBA 75% context setup also failed once, but two clean-worker
  confirmations completed the allocator call; that condition remains active
  and is not classified unusable.

## 2026-07-28 — HIL heap-artifact correction and two-board regression result

The early v7 memory classifications above were traced to the HIL transport
layout, not to evidence that the same native allocator is unusable on a
resident physical robot. V7 delivered 21–28 KB of queued allocator events in
one pre-timing setup header and materialized large output maps before sending
them. That HIL-only batching fragmented the Pololu heap. The resident native
path receives events incrementally and does not retain a complete trial
fixture.

- Setup now sends one ordered event at a time in bounded deltas.
- Device output is encoded and returned in bounded chunks without copying a
  complete indexed map.
- Host state caching now uses the canonical post-call snapshot, preventing
  false full-map retransmission.
- Preflight now starts a fresh allocator worker for every algorithm and
  calibration run, matching a clean native start.
- The corrected replay-only build is
  `micropython_1_24_o0_2539f0c4fe4d`, source-bundle SHA-256
  `2539f0c4fe4db9ca6287187ad6ff1c96f70dd9a348a9db27b27887dd76a1b268`,
  and module-set SHA-256
  `ec22e9364753a0d5d0801fcda6681caf53fea1e55820ca6a151aada4c19cf4ee`.
  Both boards received it with `main.py` unchanged.
- The corrected two-board preflight passed all 24 persistent smokes, parity,
  forced DGA context restoration, firmware/build/clock checks, safe REPL exit,
  and motor/sensor non-initialization. Both boards ran at 125 MHz; calibration
  medians were 53,550 and 53,658 microseconds, a 0.10% difference.
- All seven exact former-failure regression gates passed on their first
  generation with zero failed call phases and zero failed gate results:
  CBAA 50%, ACBBA 25%, ACBBA 50%, ACBBA 75%, PI 50%, HIPC 25%, and HIPC 50%.
  Each gate applied the target Pololu response to the live simulator and
  completed one subsequent authoritative hardware call.
- The two final requested gates were HIPC 25% (107 accepted calls, 1,923.3
  seconds wall time) and PI 50% (104 accepted calls, 2,536.5 seconds wall
  time). The already-running scheduler also completed the queued ACBBA 50%
  gate on the idle second board in 491.2 seconds.

Regression run `event_staging_and_output_streaming_v1` is complete and its
schedule is `passed`. Runner PID 29248 exited; no HIL campaign was restarted.
The next full campaign remains intentionally deferred until the user connects
an additional Pololu. V6 and V7 remain preserved as diagnostic evidence and
must not be used as representative timing or feasibility data; their formal
invalidation and the fresh V8 manifest will be completed before the next
campaign launch.

## 2026-07-28 — Third Pololu connection and three-board preflight

- Discovery found three MicroPython 1.24 Pololus at 125 MHz with no USB
  failures: COM12 `e4621cb30b2b352f`, COM13 `e4621cb30b43372f`, and COM14
  `e4621cb30b16392f`.
- COM14 initially answered the MicroPython identity probe but not the replay
  protocol because the new board did not yet contain the corrected replay
  modules. Build `micropython_1_24_o0_2539f0c4fe4d` was deployed to COM14.
  Deployment uploaded only replay `.mpy` modules and reported
  `main_py_changed=false`; existing physical robot programs were not changed.
- The corrected three-board preflight passed. All 36 allocator smoke fixtures,
  six parity checks, three forced large DGA context restores, three safe REPL
  exits, worker isolation, heap checks, and motor/sensor safety checks passed.
  Firmware, module hashes, build ID, MicroPython version, and 125 MHz clock
  matched across all boards.
- Three-repetition calibration medians were 53,827 microseconds on COM12,
  53,756 on COM13, and 53,836 on COM14. The maximum deviation from the
  reference was 0.132%, within the required 5% tolerance.
- No analysis campaign was launched during discovery or preflight.

Using 127 previously completed HIL trial wall times as the empirical planning
sample gives mean durations of 17.8 minutes for Bayesian missions and 11.3
minutes for collaborative missions. If all 1,830 planned missions completed,
the conservative workload is about 491 device-hours: roughly 10.2 days on two
boards or 6.8 days on three. Conditions that reproducibly cross the 30-second
allocator limit stop early, so the operational estimate is approximately
5.5–7 days on three boards versus 8–10.5 days on two. The third board should
save about 2.5–3.5 elapsed days, close to the ideal 33% reduction.

## 2026-07-28 — Fresh three-board V8 launch

- V6 and V7 were formally sealed `invalid_for_analysis` with reason
  `pre_timing_hil_event_batch_heap_artifact`. Their append-only raw journals
  retained their previously recorded SHA-256 hashes. Regenerated reports
  preserve 21,586 V6 and 5,400 V7 raw diagnostic attempts, while accepting
  zero calls, trials, timing samples, or feasibility classifications for
  analysis.
- V7 `CONTINUED_BY.json` and V8 `CARRY_FORWARD.json` explicitly carry zero
  result rows or classifications forward. Only the historical trial IDs and
  scenario hashes are reproduced.
- Prepared fresh campaign `pololu_native_persistent_v8` with 102 conditions
  and 1,830 mission runs. All 102 jobs began pending. The immutable manifest
  SHA-256 is
  `0ebdd6edccecfc417dc69615b9fba440d66227d9dfe91bf66cd2ed74605368d3`;
  it is bound to corrected build
  `micropython_1_24_o0_2539f0c4fe4d`.
- Launched V8 on COM12, COM13, and COM14 as background PID 32648. Initial
  longest-first assignments were Bayesian DGA 100%, DMCHBA 100%, and DGA 75%.
  Initial accepted calls on all boards had correct build identity and exact
  `allocator = filter + allocator-exclusive` timing arithmetic.
- The first three expensive conditions independently produced three confirmed
  30-second timeout attempts and stopped only their own conditions as
  `timing_unusable_30s`: Bayesian DMCHBA 100%, DGA 100%, and DGA 75%. Each
  board recovered cleanly and immediately claimed its next condition. At the
  monitored handoff there were three stopped, three running, and 96 pending
  conditions, with zero USB transport errors and an empty runner error log.
