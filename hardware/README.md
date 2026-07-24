# DTCA Benchmark Hardware

Hardware-side scripts and firmware for running decentralized task-cell allocation benchmark trials with Pololu robots, ESP32 MQTT/UART bridges, and a Jetson/host metrics hub.

## Repository Contents

- `Pololu_ACBBA.py` - Pololu MicroPython implementation of ACBBA.
- `Pololu_DMCHBA.py` - Pololu MicroPython implementation of DMCHBA.
- `esp32_DTA_BENCHMARK/esp32_DTA_BENCHMARK.ino` - ESP32 bridge between Pololu UART frames and MQTT topics.
- `metrics_hub.py` - MQTT metrics collector and trial controller.
- `trials_d1_c4_g19.csv` - generated trial target/clue scenarios.
- `trials_d1_c4_g19_readable.txt` - human-readable setup reference for the physical testbed.
- `clue_object_generator_manhat.py` - trial target/clue generator.

## Hardware Pipeline

Each robot uses:

1. A Pololu running either `Pololu_ACBBA.py` or `Pololu_DMCHBA.py`.
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
| `6` | secondary allocation payload | allocation |
| `7` | hub command to robot | control only |

Protected topics are `2` and `5`. Core topics are `1`, `2`, `4`, and `5`. Allocation topics are `3` and `6`.

## Running A Trial

1. Set the ESP32 `clientID` and `otherIDs` in `esp32_DTA_BENCHMARK.ino` for each robot.
2. Flash the ESP32 firmware.
3. Copy the selected Pololu algorithm file to each robot.
4. Confirm `metrics_hub.py` configuration:

```python
BROKER_HOST = "192.168.1.10"
ROBOT_IDS = ["00", "01", "02", "03"]
COMM_MODEL = "bernoulli"
TRIAL_MODE = "clue_search"
SCENARIO_FILE = r"...\final_trial_500.csv"
```

5. Use `trials_d1_c4_g19_readable.txt` to place the physical target and clue cells.
6. Start the robots and ESP32 bridges.
7. Run:

```bash
python metrics_hub.py
```

The hub waits for all hardcoded robots to publish pre-start position topic `1`, sends hub command `1` for pre-start, waits for a quiet window, then sends hub command `2` as trial start/t=0.

The trial ends when the hub receives the first target alert on topic `5`. The hub keeps a short drain window so messages already transmitted before peers know the target was found are still counted.

## Metrics Output

`metrics_hub.py` writes CSV files to `hub_logs/`:

- `<alg>_sys.csv` - one system-level row per trial.
- `<alg>_robots.csv` - one row per robot per trial.

System metrics include:

- `trial_id`
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
- compute time
- CPU utilization
- memory high-water metrics
- `task_cell_replans`
- `path_replans`
- `collision_prevention_events`

## Trial ID Lookup

The hub uses `SCENARIO_FILE` to map observed target/clue locations to a trial ID. It supports generator-style CSV columns:

```text
episode,object_x,object_y,clue1_x,clue1_y,...
```

Comment metadata lines beginning with `#` are ignored.

## Notes

- Hub-observed MQTT sends are treated as absolute truth.
- Receiver-side Bernoulli drop happens after hub observation, so sent-message metrics are pre-drop.
- `messages_delivered_total` and `messages_delivered_to_robot` are hub-inferred forwarding deliveries to peer robots.
- Historical simulator Gini metrics were step-based. This repo now uses `workload_gini_unique_cells_contributed` for balance of useful search coverage.
