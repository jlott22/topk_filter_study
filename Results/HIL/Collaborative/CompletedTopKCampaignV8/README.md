# Collaborative HIL Top-K Campaign V8

This is the completed collaborative portion of the mixed V8 HIL campaign.
All 48 algorithm-by-Top-K conditions are terminal: 33 completed, seven
completed with logged trial failures, and eight stopped by the 30-second
allocator-call timing guard. The successful-trial table contains 390 system
trials and the robot table contains the corresponding 1,560 rows.

The condition table is authoritative for censored coverage. Six DGA conditions
(5%-100%) and two DMCHBA conditions (75%-100%) have no successful trial rows
because the guard classified their allocator timing as unusable. Ten failed
attempts are retained in the failure log, and 35 watchdog-threshold adjustments
are retained in the adjustment log.

Use the combined trial-level CSV for successful-trial analysis and join the
condition and failure tables when reporting coverage. The verification manifest
records counts, hashes, condition-matrix checks, four-robot row checks, and
timing-arithmetic checks.
