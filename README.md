# Top-K Filtering Study

This repository contains the reproducible study record for Top-K candidate
filtering in two mission types:

- **Bayesian CLUE search:** robots search a grid after receiving probabilistic
  clues about one target.
- **Collaborative known-target visit:** robots coordinate visits to 50 known
  targets.

Start in [`results/`](results/README.md). It is organized for reviewing the
completed experiments rather than for running simulators.

## Repository map

- [`results/`](results/README.md): published campaign outputs, scenarios,
  validation evidence, and a guide to every result folder.
- [`sensitivity_suite/`](sensitivity_suite/README.md): code that prepares,
  verifies, and reports the 324-condition sensitivity campaign.
- [`simulator/`](simulator/README.md): Bayesian CLUE-search simulator only.
- [`hardware/`](hardware/README.md): current Pololu hardware programs and
  manual test workflow.
- [`allocator_replay/`](allocator_replay/README.md): host and device tooling
  for replaying allocator calls on Pololus.
- [`archive/`](archive/): superseded prototypes retained for reference.
- [`Log.md`](Log.md): chronological technical record and validation notes.

The collaborative simulator itself remains in its separate repository. It is
not needed to inspect the collaborative results committed here.
