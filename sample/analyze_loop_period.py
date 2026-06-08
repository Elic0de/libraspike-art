#!/usr/bin/env python3
import csv
import math
import os
import sys


TARGET_US = 10000
DEFAULT_TOLERANCE_US = 1000


def percentile(values, pct):
    if not values:
        return math.nan
    index = (len(values) - 1) * pct / 100.0
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return values[int(index)]
    return values[lower] + (values[upper] - values[lower]) * (index - lower)


def read_dt(path, source=None):
    values = []
    with open(path, newline="") as fp:
        header = None
        lines = []
        for line in fp:
            if header is None:
                if line.startswith("source,"):
                    header = line
                    lines.append(line)
                continue
            lines.append(line)
        if header is None:
            return values
        reader = csv.DictReader(lines)
        for row in reader:
            if source is not None and row.get("source") != source:
                continue
            dt = int(row["dt_us"])
            if dt > 0:
                values.append(dt)
    return values


def print_stats(path):
    if not os.path.exists(path):
        return
    latest_rows = {}
    with open(path, newline="") as fp:
        reader = csv.DictReader(fp)
        for row in reader:
            source = row.get("source", "unknown")
            latest_rows[source] = row

    for source, row in latest_rows.items():
        print(
            f"{source}_stats: count={row.get('count')} min={row.get('min_dt_us')} "
            f"avg={row.get('avg_dt_us')} p95={row.get('p95_dt_us')} "
            f"p99={row.get('p99_dt_us')} max={row.get('max_dt_us')} "
            f"out_of_range={row.get('out_of_range')} dropped={row.get('dropped')}"
        )


def summarize(label, values, tolerance_us):
    values = sorted(values)
    if not values:
        print(f"{label}: no samples")
        return False

    avg = sum(values) / len(values)
    min_v = values[0]
    max_v = values[-1]
    p50 = percentile(values, 50)
    p95 = percentile(values, 95)
    p99 = percentile(values, 99)
    outliers = [v for v in values if abs(v - TARGET_US) > tolerance_us]
    ok = not outliers

    print(
        f"{label}: count={len(values)} min={min_v} avg={avg:.1f} "
        f"p50={p50:.1f} p95={p95:.1f} p99={p99:.1f} max={max_v} "
        f"out_of_{TARGET_US}±{tolerance_us}us={len(outliers)}"
    )
    return ok


def main():
    if len(sys.argv) < 2 or len(sys.argv) > 4:
        print(
            "usage: analyze_loop_period.py pc_stdout.csv "
            "[spike_usb_loop_period.csv] [tolerance_us]",
            file=sys.stderr,
        )
        return 2

    tolerance_us = int(sys.argv[3]) if len(sys.argv) == 4 else DEFAULT_TOLERANCE_US
    ok = summarize("libraspike_tx", read_dt(sys.argv[1], "libraspike"), tolerance_us)

    if len(sys.argv) >= 3:
        if os.path.exists(sys.argv[2]):
            ok = summarize("spike_mot_pow_rx", read_dt(sys.argv[2], "mot_pow_rx"), tolerance_us) and ok
        else:
            print(f"spike_mot_pow_rx: skipped missing file: {sys.argv[2]}")

    print_stats("spike_usb_loop_stats.csv")

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
