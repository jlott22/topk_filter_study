# Combined HIL Top-K Campaign V8

This directory is the analysis-ready combined publication for the terminal V8
HIL campaign. It contains all 96 core conditions: 48 Bayesian non-K=1
conditions and 48 collaborative conditions. There are 1,183 successful system
trials, 4,732 robot rows, 13 failed attempts, and 36 watchdog-threshold
adjustments.

The condition table must accompany trial-level analyses because 25 conditions
were deliberately stopped by the 30-second allocator-call timing guard and nine
additional conditions completed with logged trial failures. Bayesian K=1 was
explicitly excluded from the core HIL design; it is present only in archived
diagnostic records and is not silently merged here.

The two raw call-level reports total more than 1.2 GB and remain in the ignored
local campaign tree. The compact metrics, failures, adjustments, provenance,
and validation hashes needed for analysis are published here.
