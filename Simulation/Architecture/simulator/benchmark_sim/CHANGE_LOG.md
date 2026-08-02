# Simulation and Algorithm Change Log

This document records architecture and algorithm changes made during development so the overall simulation description, algorithm explanations, and project outline can be updated later.

## How to Use This Log

For each change, add:

- Date: YYYY-MM-DD
- Area: simulation architecture, algorithm, metrics, communication, visualization, tests, or documentation
- Files changed: key files touched
- Summary: short description of what changed
- Rationale: why the change was made
- Behavior impact: how simulation or algorithm behavior changed
- Follow-up notes: open questions, assumptions, or later documentation updates needed

## Change Entries

### 2026-07-24 - Aligned Protected-Collision Timing and Empty DGA Clears

- Area: simulation architecture, collision safety, DGA, tests
- Files changed: `core/robot.py`, `algorithms/DGA.py`, `tests/test_async_movement.py`, `tests/test_dga_integration.py`, the corresponding canonical-simulator files, and hardware collision/DGA parity files
- Summary: Moved the protected current-position/intent recheck to the post-intent safety boundary and made empty DGA plans publish finite, receiver-valid prefix clears.
- Rationale: The physical protocol publishes a changed intent, turns, services communications for its settle window, and only then performs the protected recheck. The earlier simulator precheck skipped the attempted intent. DGA peers reject non-finite fitness, so an infinite-fitness empty clear could not clear a prior prefix.
- Behavior impact: When droppable peer state is absent but a protected conflict is known, the simulator now advertises the attempted transition and then the alternate transition, matching hardware. A first conflict with no alternate route emits a no-path clear, retains the goal and failure state, and makes a distinct second protected attempt before collision backoff. Empty DGA plans use fitness `0.0`, emit changed owner-prefix clears, and are accepted by receivers.
- Follow-up notes: Controlled regressions assert the protected intent sequence, two-conflict backoff, finite DGA clear reception, and parity between both simulator repositories. Physical timing and transport validation remain required.

### 2026-07-24 - Made Belief Normalization Cache-Safe and Revision Framing-Safe

- Area: algorithm, belief state, configuration, transport, tests
- Files changed: `core/belief.py`, `algorithms/base.py`, `config.py`, probability-consistency tests, and the corresponding canonical-simulator files
- Summary: Added a monotonic belief revision to every posterior rebuild and keyed the allocator's full-map maximum cache by belief object plus revision. Changed the logic-revision token to delimiter-safe `dcta_parity_v1`.
- Rationale: CPython may reuse a discarded probability dictionary's object ID, so caching only by `id(target_p)` could reuse a stale maximum after a miss or clue. The hardware UART framing uses `-` as a terminator, so a hyphenated revision token could be truncated in transit.
- Behavior impact: Every belief mutation now refreshes the exact full-map normalization maximum used by `q(c)` and therefore by `J(a,c)`. Configuration frames round-trip the complete logic revision through the existing ESP32/Pololu transport.
- Follow-up notes: Regression tests force dictionary-ID reuse-sensitive update sequences and reject delimiter-bearing configuration fields. Physical binary64, heap, and transport gates remain required.

### 2026-07-24 - Aligned Startup, Scenario, and Consensus Parity

- Area: simulation architecture, algorithm, configuration, scenarios, tests, documentation
- Files changed: `core/robot.py`, `core/scheduler.py`, `core/scenario_loader.py`, `algorithms/CBAA.py`, `algorithms/DGA.py`, `algorithms/PI.py`, `config.py`, `run_trials.py`, startup/algorithm tests, `../scenarios/final_trial_500.csv`, and `../README.md`
- Summary: Added deterministic time-zero start-cell sensing after full robot registration, strict scenario validation, ordered selection hashes with an optional required-hash preflight, an automatic cross-condition scenario-manifest lock for the canonical Top-K study profile, strict CBAA release-bid matching, PI path-head goal synchronization and exact positive-maximum normalization, exact DGA `(sender, generation, solution_id)` prediction assessment and owner-prefix delta/clear semantics, separate protected peer current-position/intent storage, duplicate searched-cell suppression, and an explicit unpublished collision-intent state.
- Rationale: Simulation runs and hardware replays need the same valid scenario set, startup observation semantics, configuration provenance, consensus goals, prediction-strike lifecycle, and collision snapshot.
- Behavior impact: Start-cell clues are now detected at time zero without a step or second visit, in the order belief update, allocator observation hook, clue publication, and allocator publication; duplicate reports of a searched peer cell no longer recompute belief; the first no-goal/no-route condition sends one protected current-cell/no-next clear while later unchanged clears remain deduplicated; Top-K study conditions cannot silently use different ordered scenario selections; targets on starts and non-integer/duplicate/out-of-bounds scenario data fail before execution; a CBAA release now requires the same winner and `local_bid <= released_bid + EPS`, so a missing/`NO_BID` field is not a wildcard; PI immediately follows a repaired path head and retains any finite positive full-map maximum even below `EPS`; distinct same-generation DGA solutions are each assessed once, unchanged owner prefixes are omitted across solution IDs, disappeared owners receive one empty clear, and exclusion remains permanent after three strikes; protected collision safety uses the advertised current cell rather than misclassifying the next intent as the current position. Target-on-start trials 172, 203, and 494 in `final_trial_500.csv` were replaced.
- Follow-up notes: The repaired scenario file raw SHA-256 is `9139f6a4fa259016f0e650489d605333758491b62151a742e406cc17dd5df085`; ordered-selection hashes are `823213c90703fd83224ad7122ee730ba64af3769ea517af252103bddd907f681` for all 500 and `33ddd00e9e07f86e272c4a946f91c9a9c4ee08ae6e902309b63caa0c5a8d5fa4` for the first 300. Headless outputs carry the logic revision plus raw and selection hashes, and the Top-K profile automatically enforces `results/topk_simulation_scenario_manifest.json`. Physical MicroPython checking, RP2040 `K=361` memory probes, and four-robot smoke trials at `K=361`, `K=18`, and an intermediate K remain required before the production campaign.

### 2026-07-24 - Promoted Memory-Bounded ACBBA, CBAA, HIPC, and PI

- Area: algorithm, performance, hardware, tests, documentation
- Files changed: `algorithms/ACBBA.py`, `algorithms/CBAA.py`, `algorithms/HIPC.py`, `algorithms/PI.py`, `algorithms/memory_optimized.py`, their compatibility `_optimized.py` modules, `tests/test_allocator_memory_optimized_equivalence.py`, `tests/benchmark_allocator_memory_optimization.py`, corresponding `../../hardware/Pololu_*.py` files, `../../hardware/allocator_memory.py`, hardware parity tests, and archived baselines
- Summary: Promoted packed candidate selection, fixed cell-indexed consensus tables, reusable route calculations, and bounded team-planning storage for ACBBA, CBAA, HIPC, and PI after archived-original parity passed.
- Rationale: The former tuple-ranked Top-K construction and growing cell-keyed dictionaries create avoidable peak allocation and fragmentation risk on RP2040 MicroPython.
- Behavior impact: Randomized decisions, paths, consensus state, messages, production Top-K 271 results, and complete four-robot simulator traces match the reference implementations. Internal representation and host execution time changed; algorithm objectives and public decisions did not.
- Follow-up notes: CPython candidate peak allocation fell by approximately 72-81%, while interpreted candidate filtering became approximately 5.6-7.9 times slower in the recorded host run. Physical RP2040 heap and timing validation remains required. Existing simulator-to-hardware scoring differences were intentionally not changed in this memory pass.

### 2026-07-24 - Promoted Compact DGA and Ported It to Pololu

- Area: algorithm, hardware, performance, tests, documentation
- Files changed: `algorithms/DGA.py`, `algorithms/DGA_optimized.py`, `tests/test_dga_optimized_equivalence.py`, `tests/dga_optimization/benchmark_dga_optimized.py`, `../../hardware/Pololu_DGA.py`, `../../hardware/test_dga_simulator_equivalence.py`, `../../archive/DGA_simulator_unoptimized.py`, `../../archive/Pololu_DGA_unoptimized.py`
- Summary: Made the compact-population implementation the simulator's standard `DGAAllocator`, archived the former simulator and Pololu defaults, and ported the compact population lifecycle to the active MicroPython implementation.
- Rationale: The duplicate produced equivalent seeded search results with substantially lower allocation pressure, so it is now the implementation used by ordinary simulator and hardware paths.
- Behavior impact: DGA parameters, objective, seed preparation, tournament selection, crossover, mutation, repair, solution selection, and messaging are unchanged. Internal populations now use 16-bit packed cell IDs and route lengths; plans are unpacked at the unchanged genetic-operator boundary.
- Follow-up notes: Desktop equivalence tests cover all operators and a complete 25-generation hardware-style search. Physical RP2040 heap and timing measurements are still required.

### 2026-07-24 - Added Compact-Population DGA Duplicate

- Area: algorithm, performance, tests, documentation
- Files changed: `algorithms/DGA_optimized.py`, `tests/test_dga_optimized_equivalence.py`, `tests/dga_optimization/benchmark_dga_optimized.py`, `../README.md`
- Summary: Preserved the reference DGA and added an optimized duplicate that stores population plans as packed unsigned cell IDs, ranks compact plans without nested signature objects, retains only the best population-size preparation plans, skips idempotent population repairs, and avoids redundant tournament/child copies.
- Rationale: The simulator-equivalent population size and generation count create excessive Python object allocation for an RP2040-oriented hardware port. The duplicate evaluates whether representation-only changes can lower memory without changing the DGA search.
- Behavior impact: Seeded tests and production-grid benchmarks produced identical candidates, goals, best plans, fitness values, generation counts, pending deltas, and ordered final populations. This entry records the evaluation stage before the later default-path promotion.
- Follow-up notes: On the controlled host benchmark, median first-allocation runtime improved by approximately 1.9-2.4 times and traced peak allocation fell by approximately 3.5-10.1 times over tested Top-K limits from 18 through 361. CPython allocation measurements are comparative and are not RP2040 heap measurements.

### 2026-07-24 - Added Host Allocator Runtime Export

- Area: metrics, simulation architecture, tests, documentation
- Files changed: `core/robot.py`, `core/scheduler.py`, `metrics/counters.py`, `metrics/summary.py`, `metrics/export.py`, `run_trials.py`, `tests/test_computational_performance.py`, `../README.md`
- Summary: Timed every allocator `choose_goal()` call with the host high-resolution clock and added per-robot aggregate latency statistics in the separate `computational_performance.csv` output.
- Rationale: Allocation performance needs to be evaluated as host computation rather than conflated with simulated mission time.
- Behavior impact: Simulation decisions and simulated timing are unchanged. Headless runs now retain allocation call count, total/mean/median/p95/maximum host latency, phase totals, total host trial runtime, and allocator host-runtime percentage.
- Follow-up notes: Host timings depend on the machine and workload running the trials, so comparisons should use a controlled execution environment.

### 2026-07-21 - Corrected Gilbert-Elliott Burst Persistence

- Area: communication, configuration, tests, documentation
- Files changed: `comms/models.py`, `run_trials.py`, `tests/test_comm_models.py`, corresponding `known_visit_sim` communication and runner files
- Summary: Reparameterized Gilbert-Elliott communication so `comm-level=q` is the stationary delivery probability and a fixed lag-one state correlation of 0.8 creates genuine GOOD/BAD runs. The transition probabilities are now `pGG=q+0.8(1-q)` and `pBB=(1-q)+0.8q`, with each directed link initialized from its stationary distribution.
- Rationale: The prior mapping `pGG=q`, `pBB=1-q` made both transition-matrix rows identical, giving zero temporal correlation and reducing the purported burst model to independent loss after initialization.
- Behavior impact: Long-run loss remains approximately `1-q`, preserving level comparability with earlier condition definitions, but losses and successes are now temporally clustered. Existing Gilbert-Elliott results were generated by the old non-bursty implementation and must be rerun before being described as burst-loss results.
- Follow-up notes: The default correlation 0.8 gives expected GOOD and BAD run lengths `1/(1-pGG)` and `1/(1-pBB)`, respectively. Verified with deterministic parameter tests and a seeded 200,000-attempt rate/autocorrelation test.

### 2026-06-20 - Made Metric Exports CSV-Only

- Area: metrics, configuration, documentation
- Files changed: `metrics/export.py`, `config.py`, `run_trials.py`, `visualization/pygame_viewer.py`, `../README.md`
- Summary: Removed optional pandas/Parquet metric export and made batch/viewer outputs CSV-only. The deprecated `--no-parquet` flag is still accepted for command compatibility but no longer changes behavior.
- Rationale: Final trial runs only need CSV outputs, so pandas/pyarrow dependencies and duplicate Parquet artifacts are unnecessary.
- Behavior impact: New runs write `trial_summary.csv`, `system_performance.csv`, `robot_performance.csv`, and `config_used.json` only. `SimConfig.write_parquet` now defaults to `False`, and run/viewer entry points force CSV-only export.
- Follow-up notes: Historical run folders may still contain `.parquet` files from earlier outputs. Verified with a one-trial CBAA smoke run that emitted only CSV metrics plus config JSON and with `python -m unittest benchmark_sim.tests.test_message_metrics benchmark_sim.tests.test_async_movement`.

### 2026-06-20 - Added Temporary Blocked-Goal Cooldown

- Area: simulation architecture, collision avoidance, tests, documentation
- Files changed: `core/robot.py`, `tests/test_async_movement.py`
- Summary: Added a narrow stuck-loop guard for repeated protected movement blocks. After two blocked attempts for the same current task goal, the robot marks that task cell temporarily invalid for its next two successful movement steps, clears the current goal, and waits a random 0-5 seconds before proceeding.
- Rationale: Under degraded communication, robots could repeatedly select a peer-occupied protected cell as the task goal, then fail movement over and over while the allocator kept choosing the same cell. The guard breaks that local loop without changing allocator reward formulas, communication behavior, assignment logic, or general path-replan accounting.
- Behavior impact: Temporary invalid task cells are exposed through the existing `blocked`/`blocked_cells` robot properties, so allocators skip them through their normal valid-task checks. The cooldown ages only on successful robot movement and expires after two moves. The random backoff uses the existing trial RNG through the message bus.
- Follow-up notes: Replaying the prior high-count `cbaa_bernoulli_050` trial 11 changed `path_replans_total` from 2786 to 53 and `collision_prevention_events` from 15 after the event cap to 20 with the new cooldown, while all robots resumed movement. Verified with `python -m unittest benchmark_sim.tests.test_async_movement benchmark_sim.tests.test_message_metrics benchmark_sim.tests.test_cbaa_entry_messages benchmark_sim.tests.test_acbba_integration benchmark_sim.tests.test_pi_integration benchmark_sim.tests.test_hipc_integration benchmark_sim.tests.test_dmchba_integration`.

### 2026-06-20 - Capped Collision Event Counting Between Moves

- Area: simulation architecture, metrics, collision avoidance, tests, documentation
- Files changed: `core/robot.py`, `tests/test_async_movement.py`
- Summary: Added a per-robot cap so `collision_prevention_events` can increment at most once after a robot's last successful move. The cap resets when the robot completes a movement step.
- Rationale: Degraded-communication trials can leave a robot repeatedly selecting a protected-blocked occupied cell as its task goal. The simulator should still count path replans for the repeated failed replanning loop, but should not log hundreds of separate collision-prevention events while the robot has not moved.
- Behavior impact: Repeated protected collision blocks before the next successful movement now produce one collision-prevention event, while `path_replans` continues to count every replan/failure. Replaying the prior high-count `cbaa_bernoulli_050` trial 11 changed system collision events from 933 to 15 while leaving `path_replans_total=2786`.
- Follow-up notes: This does not fix the underlying stuck-goal loop; it only prevents collision-event metric inflation. Verified with `python -m unittest benchmark_sim.tests.test_async_movement`.

### 2026-06-20 - Changed Drop Fraction To Unprotected Attempts

- Area: metrics, communication, tests, documentation
- Files changed: `metrics/counters.py`, `comms/bus.py`, `metrics/summary.py`, `tests/test_message_metrics.py`
- Summary: Added protected/unprotected delivered-attempt counters and changed system `message_drop_fraction` to divide dropped deliveries by unprotected delivery attempts only.
- Rationale: Protected `collision_intent` and `target` messages bypass communication loss, so including protected delivered attempts in the denominator diluted the measured drop fraction for degraded communication models.
- Behavior impact: `message_drop_fraction` is now `messages_dropped_total / (unprotected_delivered_total + messages_dropped_total)`. Existing delivered and dropped total columns are unchanged.
- Follow-up notes: Historical CSV outputs used the older all-delivered-plus-dropped denominator. Verified with `python -m unittest benchmark_sim.tests.test_message_metrics benchmark_sim.tests.test_clue_rebroadcast benchmark_sim.tests.test_async_movement`.

### 2026-06-20 - Capped DMCHBA Committed Path Horizon

- Area: algorithm, tests, documentation
- Files changed: `algorithms/DMCHBA.py`, `tests/test_dmchba_integration.py`
- Summary: Added a `COMMITMENT_HORIZON = 3` fairness cap to DMCHBA's stored/executed path after full matching-by-clone Hungarian assignment. Added debug fields that distinguish full candidate evaluation and full Hungarian assignment from the capped committed path.
- Rationale: ACBBA, PI, and HIPC are bounded to a forward multi-task horizon of 3. DMCHBA previously could keep a longer ordered path of assigned cells, giving it a possible comparability advantage through longer future commitment.
- Behavior impact: DMCHBA still evaluates every valid unsearched candidate cell and still runs the full-cell Hungarian assignment. Only this robot's committed `dmchba_path` is truncated to the first 3 ordered assigned cells. DMCHBA still sends no allocator-specific messages.
- Follow-up notes: Verified with `python -m unittest benchmark_sim.tests.test_dmchba_integration benchmark_sim.tests.test_hipc_integration`, a one-trial ideal-communication DMCHBA smoke run, and an instrumented pass confirming max candidate count 356, max assigned count 356, max committed/path length 3, post-clue DMCHBA mode reached, and zero allocation messages.

### 2026-06-20 - Tuned Rayleigh-Style Distance Loss

- Area: communication, tests, documentation
- Files changed: `comms/models.py`
- Summary: Updated the Rayleigh-style model defaults to use `tx_power_dbm=30.0`, `path_loss_ref_db=40.0`, `ref_distance=1.0`, and `sensitivity_dbm` from `--comm-level`. Increased `path_loss_exp` to `3.0` after testing so distance penalizes delivery more strongly than the benchmark baseline exponent of `2.5`.
- Rationale: The previous Rayleigh settings were too permissive on the 19x19 grid, making pilot results look close to ideal communication. The stronger exponent creates a clearer distance-dependent drop gradient while preserving received-power thresholding and per-message Rayleigh fading randomness.
- Behavior impact: Rayleigh delivery remains per non-protected message attempt and per receiver link. Protected `collision_intent` and `target` messages still bypass loss. In the fixed-distance diagnostic at `--comm-level -55`, drops increased from 1,529/50,000 with exponent 2.5 to 6,180/50,000 with exponent 3.0. A one-trial ACBBA smoke at `rayleigh_style -55` produced 104 dropped deliveries, 5,652 delivered deliveries, and `message_drop_fraction=0.0181`.
- Follow-up notes: Calibrated against the fixed diagnostic with equal samples at distances `(0,0)->(1,0)`, `(5,0)`, `(10,0)`, `(18,0)`, and `(18,18)`. Average drop targets map to approximately `--comm-level -56.04` for 10%, `--comm-level -50.63` for 25%, and `--comm-level -42.16` for 50%. Monte Carlo sanity check with 100,000 attempts per distance gave 10.04%, 25.00%, and 49.92% average drop respectively.

### 2026-06-20 - Updated Gilbert-Elliot To All-Or-Nothing Burst Loss

- Area: communication, tests, documentation
- Files changed: `comms/models.py`, `../run_pilot_30.ps1`
- Summary: Changed the Gilbert-Elliot model so GOOD-state non-protected messages always deliver and BAD-state non-protected messages always drop. Reinterpreted `--comm-level` as GOOD-state persistence (`p_good_to_good`), with BAD-state persistence set to `1.0 - comm_level`.
- Rationale: The benchmark-style GE model should represent bursty all-or-nothing link availability rather than partial delivery probabilities in each state. Higher `comm_level` now directly means higher-quality communication: links remain GOOD longer and recover from BAD faster.
- Behavior impact: `--comm-level 0.9` maps to `pGG=0.9, pBB=0.1`; `0.5` maps to `pGG=0.5, pBB=0.5`; `0.1` maps to `pGG=0.1, pBB=0.9`. Protected messages such as collision intent and target found still bypass loss in the message bus. ACBBA smoke tests showed progressive degradation in delivery rate and post-clue steps as GE level decreased.
- Follow-up notes: Existing pilot outputs using old GE labels/levels should be regenerated before comparing GE results against other communication models. Verified with `py -m unittest discover -s benchmark_sim\tests -v` and three 3-trial ACBBA CLI smoke runs. Long-run non-protected drop calibration with `pBB=1-pGG`: `--comm-level 0.90` gives about 10% drops, `0.75` gives about 25% drops, and `0.50` gives about 50% drops. A seeded 600,000-attempt diagnostic measured 10.05%, 25.11%, and 50.03% respectively.

### 2026-06-19 - Added PI and HIPC Snapshot Signature Guards

- Area: algorithms, communication, tests, documentation
- Files changed: `algorithms/PI.py`, `algorithms/HIPC.py`, `tests/test_pi_integration.py`, `tests/test_hipc_integration.py`
- Summary: Added last-sent signature checks to PI and HIPC outbound snapshot builders.
- Rationale: PI and HIPC already publish only when their pending snapshot flags are set, but a conservative dirty flag could still resend an unchanged path or bundle snapshot. Signature guards make the final send decision depend on the actual communicated allocation state.
- Behavior impact: If PI or HIPC marks a snapshot pending but the current path/bundle signature matches the last sent signature, the pending flag is cleared and no duplicate algorithm messages are published. Changed snapshots still publish normally.
- Follow-up notes: This is a defensive traffic-reduction guard and does not change task selection, bids, path construction, or receive-side consensus behavior.

### 2026-06-19 - Limited State Messages to Arrival Events

- Area: simulation architecture, communication, tests, documentation
- Files changed: `core/robot.py`, `tests/test_async_movement.py`
- Summary: Changed current-location/state publishing so robots no longer publish a state message at the start of every planning cycle. State messages are now deduplicated and published only when the robot has reached a new location.
- Rationale: Current-location messages should represent arrival at a cell, not repeated planning heartbeats. Peers should keep an agent's last communicated location until a newer location is received from that same agent.
- Behavior impact: Normal peer-position knowledge is updated only by delivered state messages after movement. Repeated attempts to publish the same current location are ignored, reducing unprotected core traffic and preventing repeated same-location updates.
- Follow-up notes: Initial robot starting positions are not broadcast as repeated state messages by the planning loop; peer knowledge begins from delivered movement-arrival messages unless initialized elsewhere.

### 2026-06-19 - Deduplicated Collision Intent Publishing

- Area: simulation architecture, communication, tests, documentation
- Files changed: `core/robot.py`, `tests/test_async_movement.py`
- Summary: Changed protected collision-intent publishing to emit only when a robot's communicated intent changes.
- Rationale: Collision intent should represent the current intended next cell and should not be repeatedly republished for every turn, intent-sync, or move sub-action when the intended cell is unchanged.
- Behavior impact: A robot now publishes a collision intent once when it commits to a next cell and publishes a replacement intent if collision prevention changes that next cell before movement. It does not publish protected clear messages after moves or cancellations. Peers keep the last communicated collision-intent location for each sender until that sender communicates a replacement.
- Follow-up notes: This should reduce protected message counts to roughly one collision-intent publication per selected cell transition, plus replacements when collision prevention changes the selected next cell.

### 2026-06-19 - Expanded Viewer Message Counters

- Area: visualization, metrics, documentation
- Files changed: `visualization/pygame_viewer.py`
- Summary: Added protected, unprotected, core, and algorithm/allocation published-message counts to the live viewer panel.
- Rationale: The viewer should distinguish guaranteed protected traffic from droppable non-protected traffic and separate core simulator messages from algorithm-specific allocation messages.
- Behavior impact: The viewer now labels total sends as `messages published`, shows `protected`/`unprotected` and `core`/`algorithm` counts, and labels delivery/drop totals as receiver outcomes.
- Follow-up notes: This is display-only; CSV metric outputs already contained these counters.

### 2026-06-18 - Implemented ACBBA Table 1 Deconfliction

- Area: algorithm, communication, tests, documentation
- Files changed: `algorithms/ACBBA.py`, `tests/test_acbba_integration.py`
- Summary: Updated ACBBA message handling to implement the full Table 1 asynchronous deconfliction protocol with explicit update, leave, reset, and timestamp-refresh actions. Added a per-cell outbound delta buffer so received table changes can be rebroadcast without requiring a local bundle snapshot.
- Rationale: ACBBA consensus should propagate changed winner/bid/timestamp information through asynchronous peer messages, including third-party claims, while avoiding full-table broadcasts and repeated unchanged rebroadcast loops.
- Behavior impact: `acbba_entry` remains the message type. The transmitting robot remains `sender`, while `winner` now represents the believed task winner and may be another robot during third-party rebroadcasts. Self-owned bundle claims still include `bundle_cells`; third-party rebroadcasts omit bundle metadata so they do not clear claims for the rebroadcasting robot. Suffix-release behavior is preserved when the robot loses a bundle task.
- Follow-up notes: Added tests for A-to-B-to-C third-party forwarding, duplicate rebroadcast suppression, leave-and-rebroadcast, reset-and-rebroadcast, update-time-and-rebroadcast, higher-bid suffix release, and first-task goal clearing.

### 2026-06-18 - Renamed PI Empty-Path Message

- Area: algorithm, communication, tests, documentation
- Files changed: `algorithms/PI.py`, `core/robot.py`, `tests/test_pi_integration.py`
- Summary: Renamed the PI empty-path clear message from `pi_snapshot` to `pi_clear_path`.
- Rationale: The message only clears stale path ownership for the sender, so the new name better describes the behavior.
- Behavior impact: PI still sends normal `pi_entry` messages for owned path cells. When the path becomes empty, it now publishes `pi_clear_path` with `path_cells=[]`, and receivers clear stale claims owned by that sender. Allocation-message counts will now report `pi_clear_path` instead of `pi_snapshot`.
- Follow-up notes: Historical outputs may still contain `pi_snapshot` topic/message counts.

### 2026-06-18 - Changed CBAA To Forward Known Table Deltas

- Area: algorithm, communication, tests, documentation
- Files changed: `algorithms/CBAA.py`, `tests/test_cbaa_entry_messages.py`
- Summary: Updated CBAA allocation messages from own-claim-only deltas to changed-known-table deltas. CBAA still keeps single-assignment ownership locally, but now forwards accepted winner/bid table changes for any cell, including claims won by another robot.
- Rationale: CBAA consensus should propagate changed known winner/bid entries through the team without broadcasting the full table.
- Behavior impact: `cbaa_entry` messages still use the transmitting robot as `sender`, but `winner` now represents the current believed cell winner and may be a different robot. Release entries use `winner=None` and include stale-owner details so receivers clear only matching stale ownership and do not erase a different stronger claim. Repeated rebroadcast loops are limited by per-cell last-sent `(cell, winner, bid)` signatures.
- Follow-up notes: Added tests for A-to-B-to-C forwarding, unchanged-forward suppression, losing the current task to a higher received bid, and safe release handling.

### 2026-06-18 - Increased Collision Intent Settle Time

- Area: simulation architecture, configuration
- Files changed: `config.py`
- Summary: Changed the default `collision_intent_settle_s` from `0.05` to `0.10`.
- Rationale: The protected collision-intent settle window should wait slightly longer before move actions when intent settling is enabled.
- Behavior impact: New runs using default configuration will allow more time for collision-intent synchronization before robots evaluate move conflicts. Tests and historical run configs with explicit overrides are unchanged.
- Follow-up notes: Historical `config_used.json` files under `runs/` still show the previous value used for those runs.

### 2026-06-18 - Reduced Initial Async Wake Spread

- Area: simulation architecture, configuration
- Files changed: `config.py`
- Summary: Changed the default `async_initial_spread_s` from `1.0` to `0.10`.
- Rationale: The default initial wake-time spread should be smaller so robot starts are less widely staggered.
- Behavior impact: New runs using default configuration will initialize robot wake events over a narrower fraction of the async step interval. Tests and historical run configs with explicit overrides are unchanged.
- Follow-up notes: Historical `config_used.json` files under `runs/` still show the old default used for those runs.

### 2026-06-17 - Added Allocation Message Rate Metrics

- Area: metrics, communication, tests, documentation
- Files changed: `metrics/counters.py`, `metrics/summary.py`, `tests/test_message_metrics.py`
- Summary: Added system-level allocation message rates: `allocation_messages_per_step`, `allocation_messages_per_post_clue_step`, and `allocation_messages_per_unique_cell`. Added `post_clue_allocation_messages_sent_total`.
- Rationale: Allocation-message overhead should be measured separately from core simulator traffic, including normalized rates over total movement, post-clue movement, and team unique-cell coverage.
- Behavior impact: System performance outputs now distinguish all post-clue messages from post-clue allocation messages. Existing `task_cell_replans_total` remains the system-level post-clue task-cell replan metric.
- Follow-up notes: `allocation_messages_per_post_clue_step` uses post-clue allocation sends divided by `post_clue_steps_to_find`.

### 2026-06-17 - Added Message Classification Metrics

- Area: metrics, communication, tests, documentation
- Files changed: `metrics/counters.py`, `comms/bus.py`, `core/robot.py`, `metrics/summary.py`, `tests/test_message_metrics.py`, `tests/test_clue_rebroadcast.py`
- Summary: Added protected/unprotected, core/allocation, per-topic, and post-clue sent-message counts at system and robot levels. Added `messages_per_post_clue_step` at the system level.
- Rationale: Communication analysis needs to separate protected safety/target traffic from degraded communication traffic and distinguish simulator core messages from algorithm allocation messages. Allocation topics are grouped together because only one task-allocation algorithm runs at a time.
- Behavior impact: New performance outputs include `protected_messages_sent_total`, `unprotected_messages_sent_total`, `core_messages_sent_total`, `allocation_messages_sent_total`, `post_clue_messages_sent_total`, `messages_per_post_clue_step`, and `messages_sent_by_topic`, plus matching per-robot sent-message classifications. Existing delivered/dropped semantics are unchanged.
- Follow-up notes: Per-topic counts are serialized as `topic:count` strings in CSV rows.

### 2026-06-17 - Gated Churn Metrics To Post-Clue Phase

- Area: metrics, simulation architecture, tests, documentation
- Files changed: `core/robot.py`, `tests/test_async_movement.py`
- Summary: Made `task_cell_replans`, `path_replans`, and `collision_prevention_events` increment only after the team's first clue has been found.
- Rationale: Churn metrics should reflect algorithm behavior after clue-driven task allocation begins, not predetermined pre-clue sweep behavior.
- Behavior impact: Pre-clue path failures, task-cell replacements, and collision-avoidance replans still affect robot behavior, but they no longer increment churn/safety metric counters. System-level totals inherit the same post-clue-only definition because they sum these counters.
- Follow-up notes: Historical CSV files under `runs/` may include whole-trial churn counts from before this change.

### 2026-06-17 - Made Unique Cell Contribution Team-Truth Based

- Area: metrics, simulation architecture, tests, documentation
- Files changed: `core/robot.py`, `metrics/counters.py`, `tests/test_async_movement.py`
- Summary: Changed `unique_cells_contributed` to increment only when the robot is the first team member to search a cell in world truth.
- Rationale: The metric should count true first-time searched cells for the team, not cells that appear new from an individual robot's local belief after communication loss.
- Behavior impact: A robot entering a globally visited cell no longer increases `unique_cells_contributed`, even if that robot did not locally know the cell had already been searched. It still increments `system_revisits_by_robot`.
- Follow-up notes: This makes the sum of per-robot `unique_cells_contributed` align with team-level `unique_cells_searched`.

### 2026-06-17 - Removed Individual Revisits Metric

- Area: metrics, documentation
- Files changed: `core/robot.py`, `metrics/counters.py`, `metrics/summary.py`
- Summary: Removed the per-robot `individual_revisits` counter and output column.
- Rationale: The project is retaining system-level revisit metrics and per-robot system revisit attribution while reducing redundant or less central search-effort metrics.
- Behavior impact: Robot performance outputs no longer include `individual_revisits`. `system_revisits_by_robot` remains unchanged.
- Follow-up notes: Historical CSV files under `runs/` may still contain `individual_revisits`.

### 2026-06-17 - Renamed Max Robot Step Metric

- Area: metrics, documentation
- Files changed: `metrics/summary.py`
- Summary: Renamed the system performance output column `max_distance_any_robot` to `max_steps_any_robot`.
- Rationale: The metric is computed from per-robot step counts, not a separate distance calculation.
- Behavior impact: New system performance outputs use the clearer `max_steps_any_robot` column.
- Follow-up notes: Historical CSV files under `runs/` may still contain `max_distance_any_robot`.

### 2026-06-17 - Removed Duplicate Search Effort Metric

- Area: metrics, documentation
- Files changed: `core/world.py`, `metrics/summary.py`
- Summary: Removed `duplicate_search_effort` from system performance output and deleted `World.duplicate_search_effort()`.
- Rationale: `duplicate_search_effort` was exactly the same value as `system_revisits`, so keeping both columns duplicated the same concept.
- Behavior impact: New system performance outputs keep `system_revisits` as the single metric for repeated team visits to already searched cells.
- Follow-up notes: Historical CSV files under `runs/` may still contain the old `duplicate_search_effort` column.

### 2026-06-17 - Distilled Churn Metrics

- Area: metrics, simulation architecture, tests, documentation
- Files changed: `core/robot.py`, `metrics/counters.py`, `metrics/summary.py`, `tests/test_async_movement.py`
- Summary: Removed `goals_selected`, `goal_churn_total`, `reservation_conflicts`, and `assignment_conflicts` from live metric output. Renamed `goal_replans` to `task_cell_replans`.
- Rationale: Churn should measure replacement of an uncompleted task cell, not normal task progression or final-state diagnostics.
- Behavior impact: `task_cell_replans` increments only when a robot selects a different task cell after its previous task cell was invalidated or abandoned before being searched. Selecting a new task cell after completing/searching the previous task cell no longer counts as churn. `path_replans` and `collision_prevention_events` remain available for path-level churn and collision-safety attribution.
- Follow-up notes: Historical CSV files under `runs/` may still contain old metric columns.

### 2026-06-17 - Removed Redundant Yield Metric

- Area: metrics, simulation architecture, documentation
- Files changed: `metrics/counters.py`, `metrics/summary.py`, `core/robot.py`, `core/scheduler.py`, `config.py`, `tests/test_async_movement.py`
- Summary: Removed the `yields` robot counter and `yield_total`/`yields` output columns. Renamed `yield_delay_s` to `replan_delay_s` because the remaining delay is used for path-failure/replan waits, not a distinct yield event.
- Rationale: After collision avoidance was changed to immediate pre-move replanning, `collision_prevention_events` became the only meaningful collision-safety counter. The `yields` counter was no longer incremented and had no distinct interpretation.
- Behavior impact: Metrics now report collision safety through `collision_prevention_events` and path changes through `path_replans`. Output schemas for new runs no longer include `yield_total` or per-robot `yields`.
- Follow-up notes: Historical CSV files under `runs/` may still contain old yield columns from earlier runs.

### 2026-06-17 - Made A* Use Droppable Peer Positions Before Protected Collision Replans

- Area: simulation architecture, collision avoidance, communication, tests
- Files changed: `core/robot.py`, `tests/test_async_movement.py`
- Summary: Changed A* planning so its initial blocked set uses only non-protected peer positions from delivered state messages plus any temporary collision block. Protected collision positions and intents no longer enter the initial A* blocked set.
- Rationale: Path planning should reflect what the robot learned through normal degraded communication first. Protected collision information should act only as a safety check immediately before executing a planned next move.
- Behavior impact: If the planned next cell conflicts with a protected peer current cell or intended next cell, the simulator records a collision-prevention event, temporarily blocks that next cell, and immediately recalls A* with the new temporary block. The event increments collision-prevention and path-replan counters without counting as a yield when a safe alternate path is found immediately.
- Follow-up notes: The collision-intent topic remains protected, but it is no longer used as proactive global path-planning knowledge.

### 2026-06-17 - Removed Selected-Cell Communication Channel

- Area: simulation architecture, communication, algorithms, documentation
- Files changed: `core/robot.py`, `algorithms/base.py`, `algorithms/Auction_greedy.py`, `algorithms/DMCHBA.py`, `algorithms/HIPC.py`, `algorithms/CBAA.py`
- Summary: Removed simulator-level broadcasting and receiving of selected task cells. Auction-Greedy now estimates winnable task cells from peer locations only, without using peer-selected cells as reservations. Removed peer-selected-cell compatibility fallbacks from DMCHBA and HIPC.
- Rationale: Selected task-cell sharing should not be a communication channel in the simulation. Auction-Greedy should be an implicit can-win allocator that infers competition from peer locations rather than announced selected cells.
- Behavior impact: Robots still maintain an internal `current_goal` movement target, but that target is no longer published to peers. Algorithms that need allocation communication must use their own explicit algorithm messages. Auction-Greedy may now choose cells that peers are also moving toward if peer positions alone do not make those cells unwinnable.
- Follow-up notes: Documentation should describe selected cells as internal task cells, not communicated selected-cell reservations.

### 2026-06-17 - Change Log Created

- Area: documentation
- Files changed: `CHANGE_LOG.md`
- Summary: Added a standing change log for future simulation architecture and algorithm updates.
- Rationale: The project needs a durable record of implementation changes so the simulation overview, algorithm descriptions, and project outline can be revised accurately.
- Behavior impact: No runtime behavior changed.
- Follow-up notes: Add an entry after each future architecture or algorithm change.
