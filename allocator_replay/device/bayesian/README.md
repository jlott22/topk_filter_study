# Bayesian allocator ports

Replay modules named `replay_b_<algorithm>` are generated from the current
`simulator/benchmark_sim/algorithms` sources by
`allocator_replay.device.build`. Generated `.py` and `.mpy` files live under
`results/allocator_replay/device_build/<build-id>/` so authoritative simulator
logic is never copied here and allowed to drift.

The generator redirects only portability dependencies (types, timing, random
state, compact maps, copying, and hashing). It does not import or modify any
`hardware/Pololu_*.py` program.
