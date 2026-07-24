from __future__ import annotations

import argparse
import subprocess
import time

from benchmark_sim.tests.run_coverage_horizon_sensitivity import main as run_coverage_horizon
from benchmark_sim.tests.run_current_sensitivity_reruns import SleepInhibitor, log


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run coverage horizon sensitivity after another process exits.")
    parser.add_argument("--wait-pid", type=int, required=True)
    return parser.parse_args()


def wait_for_pid(pid: int) -> None:
    log(f"Waiting for PID {pid} before starting coverage horizon sensitivity.")
    while True:
        result = subprocess.run(
            [
                "powershell.exe",
                "-NoProfile",
                "-Command",
                f"if (Get-Process -Id {pid} -ErrorAction SilentlyContinue) {{ exit 0 }} else {{ exit 1 }}",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if result.returncode != 0:
            log(f"PID {pid} is no longer running; starting coverage horizon sensitivity.")
            return
        time.sleep(60)


def main() -> None:
    args = parse_args()
    with SleepInhibitor():
        wait_for_pid(args.wait_pid)
        run_coverage_horizon()


if __name__ == "__main__":
    main()
