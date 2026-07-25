# ACBBA, CBAA, HIPC, and PI Memory-Optimization Comparison

Date: 2026-07-24

This comparison used CPython `tracemalloc` and `perf_counter_ns` on the
repository host. It compares each archived/reference simulator implementation
with the promoted memory-bounded default on a 19 by 19 grid, four robots, and
Top-K 271. These measurements compare implementations on this host; they are
not RP2040 heap or timing measurements.

Command:

```text
cd simulator
python -m benchmark_sim.tests.benchmark_allocator_memory_optimization
```

| Algorithm | Candidate time old/new (ms) | Candidate traced peak old/new (bytes) | First allocation time old/new (ms) | First allocation traced peak old/new (bytes) |
|---|---:|---:|---:|---:|
| ACBBA | 3.243 / 22.626 | 63,924 / 18,112 | 26.980 / 56.282 | 65,388 / 44,936 |
| CBAA | 3.216 / 22.977 | 63,924 / 18,112 | 4.933 / 30.148 | 64,972 / 41,158 |
| HIPC | 3.966 / 22.241 | 94,332 / 18,112 | 36.426 / 126.141 | 96,236 / 39,467 |
| PI | 3.011 / 23.519 | 63,924 / 18,112 | 48.592 / 68.342 | 66,780 / 36,972 |

The reusable packed selector reduced traced candidate-call peak allocation by
approximately 72% for ACBBA, CBAA, and PI and 81% for HIPC. First-allocation
peak reduction was approximately 31% for ACBBA, 37% for CBAA, 59% for HIPC,
and 45% for PI.

The tradeoff is higher interpreted runtime. The packed selector performs its
ordering in Python instead of using CPython's C-level tuple sort. This mirrors
the accepted DMCHBA memory-for-time tradeoff and prioritizes bounded
MicroPython allocation.

Dense cell tables also increase retained storage when only one to three claims
exist. After one sparse allocation, retained host object size was higher in
the optimized implementation. This is intentional: fixed tables reserve a
bounded worst-case capacity at initialization and prevent later dictionary
growth and tuple-key fragmentation. Cold first-allocation peak still fell for
all four algorithms because large transient candidate and planning object
graphs were removed.

Behavioral validation:

- 12 randomized simulator cases per algorithm across candidate limits `all`,
  5, 12, and 25;
- production 19 by 19, Top-K 271 cases;
- repeated allocation after completing the first goal;
- exact consensus table and outbound message equality;
- complete four-robot event-trace equality;
- randomized hardware-extraction cases across Top-K 20%, 35%, 50%, 75%, and
  100%;
- hardware path, table, and emitted UART payload equality;
- the complete simulator suite: 114 tests passed;
- the complete hardware suite: 17 tests passed.

Physical RP2040 `gc.mem_free()` and allocator timing measurements remain
required.
