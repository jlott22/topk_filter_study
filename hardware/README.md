# DTCA Benchmark Hardware

Hardware-side scripts and firmware for running decentralized task-cell allocation benchmark trials with Pololu robots, ESP32 MQTT/UART bridges, and a Jetson/host metrics hub.

## Current program status

- `Pololu_*.py` contains temporary programs copied from the benchmark
  hardware repository because those versions are known to boot.
  Use these for physical startup, communication, motion, and hub testing while
  the memory problem is investigated. They accept the hub's six Top-K study
  values but retain the older benchmark allocator logic, so they are not valid
  simulator-parity study implementations.
- `../archive/in_the_works/Pololu_*.py` contains the archived
  Top-K/simulator-parity implementations. They are preserved for memory
  optimization and desktop parity testing, but must not be deployed for data
  collection until physical K=361 startup and allocator smoke tests pass.
- `metrics_hub.py` remains the controller for both sets and now restricts
  hardware runs to the manually selected five-scenario cohort. The temporary
  programs implement its configuration and command acknowledgment protocol
  themselves.

## Repository Contents

- `Pololu_*.py` - temporary known-bootable robot programs with a lightweight
  Top-K candidate prefilter.
- `../archive/in_the_works/` - archived memory-heavy Top-K/parity robot
  programs.
- `esp32_DTA_BENCHMARK/esp32_DTA_BENCHMARK.ino` - ESP32 bridge between Pololu UART frames and MQTT topics.
- `metrics_hub.py` - MQTT metrics collector and trial controller.
- `../simulator/scenarios/final_trial_500.csv` - canonical validated study
  scenario manifest used by the hub by default.
- `../results/hardware_pilot/legacy_clue_trials_g19_c4*` - historical generated trial artifacts; these
  are not the hub's default study input.
- `clue_object_generator_manhat.py` - trial target/clue generator.

## Hardware Pipeline

Each robot uses:

1. A Pololu running the selected `Pololu_*.py` temporary allocator or, after
   memory acceptance, an archived `../archive/in_the_works/Pololu_*.py` study
   allocator.
2. An ESP32 running `esp32_DTA_BENCHMARK.ino`.
3. MQTT broker reachable at `192.168.1.10:1883`.
4. A host/Jetson running `metrics_hub.py`.

The Pololu sends UART frames to the ESP32 as:

```text
<topic>.<payload>-
```

The ESP32 publishes those frames to MQTT topics:

```text
<robot_id><topic>
```

Example: robot `02` publishing position on topic `1` uses MQTT topic `021`.

## Topic Map

| Topic | Meaning | Simulator Category |
|---|---|---|
| `1` | robot state / position | `state` |
| `2` | next-step collision intent | `collision_intent` |
| `3` | primary allocation payload | allocation |
| `4` | clue report | `clue` |
| `5` | target alert | `target` |
| `6` | configuration or control acknowledgment | control only |
| `7` | hub command to robot | control only |

Protected topics are `2` and `5`. Core topics are `1`, `2`, `4`, and `5`.
Allocation traffic uses topic `3`; topic `6` is excluded from trial-message
metrics even when its payload is malformed.

## Running A Trial

1. Set the ESP32 `clientID` and `otherIDs` in `esp32_DTA_BENCHMARK.ino` for each robot.
2. Flash the ESP32 firmware.
3. For temporary hardware testing, copy the selected
   `Pololu_<ALGORITHM>.py` file to each robot as its actual startup
   `main.py`. Set `ROBOT_ID` separately to `00`, `01`, `02`, and `03`.
   Each program is self-contained and no second Python module is required.
4. Confirm `metrics_hub.py` configuration:

```python
BROKER_HOST = "192.168.1.10"
ROBOT_IDS = ["00", "01", "02", "03"]
COMM_MODEL = "bernoulli"
TRIAL_MODE = "clue_search"
SCENARIO_FILE = r"..\simulator\scenarios\final_trial_500.csv"
STUDY_MANIFEST_LOCK = r"..\results\hardware_handpicked_5_scenario_manifest.json"
```

The canonical simulator start cells are `00=(0,0)`, `01=(0,6)`,
`02=(0,12)`, and `03=(0,18)`, all facing east. The hub will not run the
Top-K hardware profile with a different grid, robot set, trial mode, horizon,
logic revision, or a scenario whose target is on one of these starts.

The temporary programs retain the older benchmark allocator logic. They apply
the Top-K candidate count from the hub and acknowledge the required
`dcta_parity_v1` field solely as a transport-compatibility token. The accepted
rates are `1`, `.75`, `.5`, `.25`, `.10`, and `.05`, corresponding to K=361,
271, 181, 90, 36, and 18. Data from these files must be labeled as temporary
hardware testing, not as simulator-parity study results.

Keep temporary host output and its scenario lock separate from study files:

```powershell
python hardware\metrics_hub.py --algorithm ACBBA --top-k-rate .10 --drop-rate 0 `
  --out-dir hardware\hub_logs\temporary_benchmark `
  --scenario-manifest-lock results\temporary_benchmark_manifest.json
```

Replace `ACBBA` with the flashed algorithm. Temporary robot-local metrics use
`metrics-temp-<ALGORITHM>.txt`. Because benchmark DGA and DMCHBA use
allocation-heavy dynamic structures, physically smoke-test K=18 first and
increase K while monitoring for runtime memory failures.

5. Choose one of the five handpicked study scenarios when prompted. The hub
   shows its target and clue cells and publishes the selected row as JSON on
   `hub/trial_task`.
6. Start the robots and ESP32 bridges.
7. Run:

```bash
python hardware/metrics_hub.py --algorithm ACBBA --trials 1
```

The hub is manual-only. It waits for all configured robots to return home,
prompts for the study trial ID, Top-K rate, and drop rate, requires placement
confirmation, and then broadcasts:

```text
CFG,<sequence>,<top_k_ppm>,<top_k_cells>,<drop_ppm>,clue_search,<horizon>,dcta_parity_v1,<scenario_sha256>
```

Each robot resizes its allocator workspace and replies on topic `6` with its
flashed algorithm and all applied values:

```text
CFGACK,<sequence>,<algorithm>,<top_k_ppm>,<top_k_cells>,<drop_ppm>,clue_search,<horizon>,dcta_parity_v1,<scenario_sha256>,<status>
```

The effective horizon is `1` for CBAA and `3` for the other five algorithms.
The hub will not start until every robot reports a matching `OK`. Trial control
then uses the same applied configuration sequence:

```text
CMD,PRESTART,<sequence>
CMD,START,<sequence>
CMD,RUN,<sequence>
CMD,ABORT,<sequence>
```

Each robot replies on topic `6`:

```text
CMDACK,<sequence>,<robot_id>,READY
CMDACK,<sequence>,<robot_id>,STARTED
CMDACK,<sequence>,<robot_id>,RUNNING
CMDACK,<sequence>,<robot_id>,ABORTED
```

The hub retries application commands, requires all `READY` replies before the
quiet window, requires all `STARTED` replies to prove that every robot is
stationary and armed, and only then publishes `RUN`. The initial `RUN` publish
atomically establishes hub trial time zero and the pending-event replay
boundary. The trial becomes active only after every robot reports `RUNNING`.

A robot accepts only the sequence it actually applied. Duplicate commands
re-ACK without repeating the transition. Only the first valid `START` clears
pre-trial peer/intent caches and resets onboard trial counters; it does not
release motion. The first valid `RUN` stamps the onboard metric start time and
releases the already-armed search without clearing any knowledge or counters.
While waiting for its own `RUN` copy, an armed robot accepts, applies, counts,
and forwards normal trial traffic from peers that have already received
`RUN`. This preserves one-time clues and allocator state across staggered
command delivery.

A target received any time after `PRESTART` and before full `RUNNING` quorum
invalidates and aborts the trial instead of producing a completed data row.
Non-target frames received after the initial `RUN` publish are buffered in
arrival order and replayed exactly once after quorum so early steps,
start-cell clues, and allocator traffic are not lost. `ABORT` is accepted by
configured, ready, armed, and running robots.

Before connecting to the robots, the hub loads and validates the fixed
five-scenario cohort. By default it creates or checks the repository-shared
`results/hardware_handpicked_5_scenario_manifest.json`; a later algorithm or
Top-K condition is refused if the study/source ID mapping or scenario
contents differ.
Use `--expected-scenario-sha256 <hash>` for an additional explicit check.
`--scenario-manifest-lock <path>` is available for a deliberately separate
smoke-test or study cohort; every condition within that cohort must use the
same lock path and exact selection.

Runs default to Top-K `1.0` (361 cells), drop rate `0.0`, and one manually
operated trial. `--trials N` keeps the hub open for N manual trials. Pressing
Enter at the trial-ID, Top-K, or drop-rate prompt reuses the previous value,
so the same physical layout can remain in place while Top-K values are swept.
After every started trial, the hub requires a yes/no response indicating
whether any robot experienced a memory error.

Before trial start, the terminal prints each robot's connection, home-ready
state, and acknowledged algorithm/Top-K/drop-rate configuration. During an
active trial, routine state and allocation traffic stays silent; only clue and
target detections are printed with the reporting robot and cell.

If a robot crashes and cannot send a target alert, press `M` once without
pressing Enter. The hub freezes logical metrics at that instant, sends the
normal `ABORT` command to surviving robots, drains in-flight traffic, and asks
whether a memory error occurred. Answering yes records `memory_error=1` and
`trial_status=memory_error_crash`; answering no records `memory_error=0` and
`trial_status=manual_stop`. The partial trial is retained in both system and
per-robot CSVs instead of leaving the hub waiting for the target timeout.
`Ctrl+C` remains the fallback when the host terminal does not support
single-key input.

The trial ends when the hub receives the first target alert on topic `5`. A
physical bump is counted as the same terminal logical cell-entry step used by
the simulator, while the robot retains its real pre-bump pose for retreat.
Logical metrics freeze at the first alert. The hub keeps a short drain window
for auditing messages already in flight, but those messages cannot add search
steps after termination.

## Metrics Output

`metrics_hub.py` writes CSV files to `hub_logs/`:

- `<alg>_sys.csv` - one system-level row per trial.
- `<alg>_robots.csv` - one row per robot per trial.

System metrics include:

- `trial_id`
- `source_trial_id`
- `trial_mode`
- `algorithm`
- `comm_model`
- `comm_level`
- `total_team_steps`
- `steps_before_first_clue`
- `post_clue_steps_to_find`
- `unique_cells_searched`
- `system_revisits`
- `messages_sent_total`
- `messages_delivered_total`
- `protected_messages_sent_total`
- `unprotected_messages_sent_total`
- `core_messages_sent_total`
- `allocation_messages_sent_total`
- `post_clue_messages_sent_total`
- `post_clue_allocation_messages_sent_total`
- `messages_per_unique_cell`
- `messages_per_post_clue_step`
- `allocation_messages_per_step`
- `allocation_messages_per_post_clue_step`
- `allocation_messages_per_unique_cell`
- `messages_sent_by_topic`
- `max_steps_any_robot`
- `max_messages_any_robot`
- `workload_gini_unique_cells_contributed`

Robot metrics include:

- `trial_id`
- `source_trial_id`
- `trial_mode`
- `algorithm`
- `comm_model`
- `comm_level`
- `robot_id`
- `steps_total`
- `steps_after_first_clue`
- `unique_cells_contributed`
- `system_revisits_by_robot`
- `messages_sent`
- `protected_messages_sent`
- `unprotected_messages_sent`
- `core_messages_sent`
- `allocation_messages_sent`
- `post_clue_messages_sent`
- `post_clue_allocation_messages_sent`
- `messages_sent_by_topic`
- `messages_delivered_to_robot`

Each Pololu also writes local robot metrics, including:

- per-topic sent/received counts
- bytes sent/received
- motor time
- CPU utilization
- memory high-water metrics
- candidate-filter call count and total, mean, and maximum time
- allocator solve time excluding measured candidate-filter time, with total,
  mean, and maximum
- allocator call count and total, mean, and maximum allocation-decision time
- allocator time as a percentage of trial wall time
- mean step time, calculated as trial wall time divided by robot step count
- `task_cell_replans`
- `path_replans`
- `collision_prevention_events`

Candidate-filter and allocator times are reported in microseconds. The onboard
CSV fields are `candidate_filter_calls`, `candidate_filter_time_us_total`,
`candidate_filter_time_us_mean`, `candidate_filter_time_us_max`,
`allocator_solve_time_us_total`, `allocator_solve_time_us_mean`,
`allocator_solve_time_us_max`, `allocator_calls`,
`allocator_time_us_total`, `allocator_time_us_mean`,
`allocator_time_us_max`, `allocator_time_pct`, `trial_time_ms`, and
`mean_step_time_ms`. Every onboard metrics row also records the deployed
`top_k_rate`, rounded `top_k_max_cells`, `drop_rate`, and
`config_sequence`, plus `trial_mode`, effective `commitment_horizon`,
`logic_revision`, and `scenario_sha256`, so imported hardware results remain
self-describing. The hub's system and per-robot CSVs record the renumbered
`trial_id`, original CSV `source_trial_id`, acknowledged algorithm and
configuration, `algorithm_verified`, the manual `memory_error` flag, and the
trial status. System rows also retain expected and reported targets, expected
and observed clues, unexpected clues, and location warnings. A mismatch is
warned and audited but does not change a completed trial's status. Confirmed
manually stopped memory crashes use the explicit `memory_error_crash` status.
Configuration acknowledgments are retained separately in
`<algorithm>_configuration_acks.csv`.

There is no unattended mode. Trial selection, placement confirmation,
condition confirmation, and memory-error reporting always require an
operator. The deprecated `--comm-level` option remains an alias for
`--drop-rate`.

## Handpicked Trial IDs

The hub loads these five rows from `SCENARIO_FILE` and renumbers them for the
hardware study:

| Study trial ID | Source episode | Target |
|---:|---:|---:|
| 1 | 4 | `(5,9)` |
| 2 | 53 | `(7,2)` |
| 3 | 232 | `(3,11)` |
| 4 | 394 | `(5,4)` |
| 5 | 473 | `(13,16)` |

The operator selects the study ID before every trial. Repeated IDs are
allowed within one invocation. The hub supports generator-style CSV columns:

```text
episode,object_x,object_y,clue1_x,clue1_y,...
```

Comment metadata lines beginning with `#` are ignored.

Scenario loading is fail-fast: all five source episodes must exist,
coordinates must be complete integers inside
the 19x19 grid; trial IDs and clue coordinates must be unique; a target cannot
also be a clue or a robot start. A clue is allowed on a start and is detected
before the first movement step. The selected scenario list is hashed and that
hash must be acknowledged by every robot. The canonical input is
`../simulator/scenarios/final_trial_500.csv`; the similarly named files under
`../results/` are retained historical artifacts rather than default inputs.

The remapped five-scenario cohort SHA-256 is
`92ebcdc84dc259fc27fc6123bef9ca9f0488a874e84e405344e349aa2d07d393`.
Keep its manifest lock unchanged across every algorithm and Top-K condition.

## Pre-Campaign Hardware Gates

Desktop parity tests cannot establish RP2040 precision, heap headroom, motor
timing, radio ordering, or four-robot collision behavior. Before the
300-trial simulation campaign is treated as final, complete and record these
physical gates in `Log.md`:

1. Parse or compile every archived
   `../archive/in_the_works/Pololu_*.py` file with the chosen
   MicroPython-compatible checker.
2. Boot every robot with the deployed firmware and confirm that the shared
   `require_binary64()` startup probe succeeds. The allocator intentionally
   stops if scalar floats or `array('d')` cannot preserve binary64 precision;
   do not bypass this check or fall back to the former quantized protocol.
   The upstream RP2 port currently defaults to single-precision floats, so the
   deployed Pololu build must be explicitly verified and may require a custom
   double-precision MicroPython build.
3. Run each allocator's maximum-memory probe at Top-K `1.0` (`K=361`) on the
   actual RP2040, with no `MemoryError`, and record minimum free heap/headroom
   after loading the double-precision firmware and production allocator.
4. Run controlled four-robot smoke trials for all six algorithms at `K=361`,
   `K=18`, and at least one intermediate K.
5. Verify every robot acknowledges the exact algorithm, rate/K, drop rate,
   mode, effective horizon, logic revision, and selected-scenario hash.
6. Confirm decoded intents/clears, allocator triggers, target terminal-step
   accounting, and onboard metric rows before starting production collection.
7. Stress the deployed UART/MQTT chain with the largest DGA message burst
   while a receiver is inside its longest measured allocation. The Pololu
   programs now request `rxbuf=4096`, `txbuf=1024`, `timeout=1000`, and
   `timeout_char=10`; drain RX in at most 256-byte chunks; discard and
   delimiter-resynchronize frames longer than 256 bytes; and serialize shared
   frame construction through a locked, bounded write-all loop. Short,
   `None`, and zero-byte writes are retried; a deadline failure raises visibly
   and does not increment sent-frame metrics. Confirm this full buffer
   allocation fits at `K=361` rather than reducing it silently.
8. During every smoke run, confirm the hub receives all sequenced `READY`,
   `STARTED`, and `RUNNING` acknowledgments; no robot moves at `START`; every
   robot is `STARTED` before the first `RUN` publish; retry/deduplication works;
   and an intentionally stale sequence is rejected. The application ACK is
   authoritative; the transparent ESP bridge remains compatible and MQTT QoS
   0 is acceptable because missing commands are retried at the application
   layer.

## Notes

- Hub-observed MQTT sends are treated as absolute truth.
- Receiver-side Bernoulli drop happens after hub observation, so sent-message metrics are pre-drop.
- `messages_delivered_total` and `messages_delivered_to_robot` are hub-inferred forwarding deliveries to peer robots.
- Historical simulator Gini metrics were step-based. This repo now uses `workload_gini_unique_cells_contributed` for balance of useful search coverage.
