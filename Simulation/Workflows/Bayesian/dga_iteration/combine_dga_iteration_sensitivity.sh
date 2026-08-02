#!/usr/bin/env bash
set -euo pipefail

# Recombine DGA iteration sensitivity raw outputs, including repair outputs.
# Adds source metadata and de-duplicates by source_iter/source_comm_label/episode.

cd /home/jlott/dcta_benchmark_sim

OUT_ROOT="runs/sensitivity_dga_iterations"
COMBINED_DIR="$OUT_ROOT/combined"
mkdir -p "$COMBINED_DIR"

python3 - <<'PY'
import csv
from pathlib import Path

out_root = Path("runs/sensitivity_dga_iterations")
combined_dir = out_root / "combined"
combined_dir.mkdir(parents=True, exist_ok=True)

filenames = ["system_performance.csv", "trial_summary.csv", "robot_performance.csv"]
metadata_cols = [
    "source_iter",
    "source_comm_label",
    "source_part",
    "source_is_repair",
    "source_episode",
    "source_out_dir",
]

def parse_source(path: Path):
    parts = path.parts
    source_iter = ""
    source_comm_label = ""
    source_part = ""
    try:
        i = next(idx for idx, p in enumerate(parts) if p.startswith("iter_"))
        source_iter = parts[i].replace("iter_", "", 1)
        if i + 1 < len(parts):
            source_comm_label = parts[i + 1]
    except StopIteration:
        pass
    for p in parts:
        if p.startswith("part_"):
            source_part = p.replace("part_", "", 1)
    source_is_repair = "1" if any(p.startswith("repair") for p in parts) else "0"
    source_out_dir = str(path.parent)
    return {
        "source_iter": source_iter,
        "source_comm_label": source_comm_label,
        "source_part": source_part,
        "source_is_repair": source_is_repair,
        "source_out_dir": source_out_dir,
    }

def sort_key(path: Path):
    meta = parse_source(path)
    # Read original data first and repair data second, so repair wins if duplicate keys exist.
    return (int(meta["source_is_repair"]), str(path))

def get_episode_id(row: dict):
    # Primary scenario identifier is episode. Fall back to trial_id only for older raw outputs.
    return row.get("episode", "") or row.get("trial_id", "")

def row_key(filename: str, meta: dict, row: dict):
    eid = get_episode_id(row)
    base = (meta.get("source_iter", ""), meta.get("source_comm_label", ""), eid)
    if filename == "robot_performance.csv":
        return base + (row.get("robot_id", ""),)
    return base

manifest_rows = []

for filename in filenames:
    files = []
    for p in out_root.rglob(filename):
        if combined_dir in p.parents:
            continue
        files.append(p)
    files = sorted(files, key=sort_key)

    out_path = combined_dir / filename
    if not files:
        print(f"[SKIP] no {filename} files found")
        continue

    rows_by_key = {}
    original_fieldnames = None
    total_input_rows = 0

    for file_path in files:
        meta = parse_source(file_path)
        file_rows = 0
        try:
            with file_path.open(newline="") as f:
                reader = csv.DictReader(f)
                if not reader.fieldnames:
                    continue
                if original_fieldnames is None:
                    original_fieldnames = [c for c in reader.fieldnames if c not in metadata_cols]
                for row in reader:
                    # Strip old metadata columns if this script is run on previously labeled files by mistake.
                    clean_row = {k: v for k, v in row.items() if k not in metadata_cols}
                    key = row_key(filename, meta, clean_row)
                    combined_row = dict(clean_row)
                    combined_row.update(meta)
                    combined_row["source_episode"] = get_episode_id(clean_row)
                    rows_by_key[key] = combined_row
                    file_rows += 1
                    total_input_rows += 1
        except Exception as exc:
            print(f"[WARN] could not read {file_path}: {exc}")
            continue

        manifest_rows.append({
            "filename": filename,
            "source_iter": meta["source_iter"],
            "source_comm_label": meta["source_comm_label"],
            "source_part": meta["source_part"],
            "source_is_repair": meta["source_is_repair"],
            "rows": str(file_rows),
            "source_out_dir": meta["source_out_dir"],
        })

    if original_fieldnames is None:
        print(f"[SKIP] no readable {filename} files found")
        continue

    fieldnames = original_fieldnames + metadata_cols
    rows = list(rows_by_key.values())
    rows.sort(key=lambda r: (
        int(r.get("source_iter") or 0),
        r.get("source_comm_label", ""),
        int(get_episode_id(r) or 0) if str(get_episode_id(r)).isdigit() else str(get_episode_id(r)),
        int(r.get("robot_id") or 0) if str(r.get("robot_id", "")).isdigit() else str(r.get("robot_id", "")),
    ))

    with out_path.open("w", newline="") as out:
        writer = csv.DictWriter(out, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)

    duplicates_removed = total_input_rows - len(rows)
    print(f"[OK] combined {filename}: input_rows={total_input_rows} output_rows={len(rows)} duplicates_removed={duplicates_removed}")

manifest_path = combined_dir / "source_manifest.csv"
with manifest_path.open("w", newline="") as out:
    fieldnames = ["filename", "source_iter", "source_comm_label", "source_part", "source_is_repair", "rows", "source_out_dir"]
    writer = csv.DictWriter(out, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(manifest_rows)
print(f"[OK] wrote {manifest_path}")
PY

echo
echo "[DONE] Recombined labeled/deduplicated CSVs saved in:"
echo "$COMBINED_DIR"
