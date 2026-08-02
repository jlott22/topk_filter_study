from __future__ import annotations

from .core import run_campaign, workers_arg


def main() -> None:
    args = workers_arg().parse_args()
    run_campaign(args.workers)


if __name__ == "__main__":
    main()

