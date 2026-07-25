# Decentralized Allocator Optimization and Equivalence Log

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
  `results/allocator_memory_optimization_comparison.md`

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
