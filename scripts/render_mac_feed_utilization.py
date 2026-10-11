#!/usr/bin/env python3
"""Render the accepted V2/V3 utilization table; no simulation or EDA execution.

Source: sim/README.md#utilization-measurement. Matplotlib 3.9.2 was used for
the checked-in SVG. Run from any directory; --output accepts a preview directory.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "sim/README.md"


def measurements():
    section = SOURCE.read_text().split("## Utilization measurement", 1)[1].split("\n## ", 1)[0]
    pattern = r"^\| (V[23]) \| (\d+) \| (\d+) \| (\d+) \| ([\d.]+)% \|$"
    rows = []
    for version, outstanding, pairs, cycles, reported in re.findall(pattern, section, re.M):
        pairs, cycles = int(pairs), int(cycles)
        utilization = 100 * pairs / cycles
        if round(utilization, 2) != float(reported):
            raise ValueError("Published utilization disagrees with accepted cycle counts")
        rows.append(dict(design=version, outstanding_bursts=int(outstanding),
                         accepted_pairs=pairs, dma_busy_bus_cycles=cycles,
                         utilization_percent=utilization, reported_percent=float(reported)))
    if [r["design"] for r in rows] != ["V2", "V3"] or any(r["accepted_pairs"] != 256 for r in rows):
        raise ValueError("Expected the accepted 256-element V2/V3 comparison")
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "docs/figures")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rows = measurements()
    gain = rows[1]["utilization_percent"] / rows[0]["utilization_percent"]
    plt.rcdefaults()
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11,
                         "text.color": "#182c43", "svg.fonttype": "none",
                         "svg.hashsalt": "vector-mac-feed-utilization",
                         "axes.spines.top": False, "axes.spines.right": False})
    fig = plt.figure(figsize=(10, 3.5), facecolor="white")
    fig.text(.035, .90, "Hide memory latency with outstanding reads", size=18, weight="bold")
    fig.text(.035, .81, "256 elements/vector · 16-beat bursts · 100-cycle first-beat latency",
             size=10, color="#52677e")
    ax = fig.add_axes([.23, .26, .67, .46])
    positions = [1, 0]
    ax.barh(positions, [r["utilization_percent"] for r in rows], height=.42,
            color=["#72849a", "#2563a7"], zorder=3)
    ax.set_yticks(positions, [f'{r["design"]} · {r["outstanding_bursts"]} outstanding' for r in rows])
    ax.set_xlim(0, 34)
    ax.set_ylim(-.55, 1.55)
    ax.set_xticks([0, 10, 20, 30])
    ax.set_xlabel("Accepted operand pairs / DMA-busy bus cycle (%)", color="#52677e", size=10)
    ax.tick_params(length=0, colors="#52677e", pad=8)
    ax.spines["left"].set_visible(False)
    ax.spines["bottom"].set_color("#cbd7e3")
    ax.grid(axis="x", color="#e7edf3", zorder=0)
    for y, row in zip(positions, rows):
        ax.text(row["utilization_percent"] + .6, y, f'{row["reported_percent"]:.2f}%',
                va="center", size=13, weight="bold")
        ax.text(.2, y - .37, f'{row["accepted_pairs"]} pairs / {row["dma_busy_bus_cycles"]:,} busy cycles',
                size=9, color="#52677e")
    fig.text(.035, .075, f"{gain:.1f}× simulated MAC-feed utilization", size=12, weight="bold", color="#2563a7")
    fig.text(.035, .015, "Source: sim/README.md · Operand delivery, not MAC PE activity or whole-system speedup.",
             size=9, color="#52677e")
    output = args.output / "mac_feed_utilization.svg"
    fig.savefig(output, metadata={"Date": None})
    plt.close(fig)
    svg = output.read_text().replace("'DejaVu Sans'", "Arial, 'DejaVu Sans', sans-serif")
    output.write_text("\n".join(line.rstrip() for line in svg.splitlines()) + "\n")
    manifest = dict(source="sim/README.md#utilization-measurement",
                    source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
                    generator="scripts/render_mac_feed_utilization.py",
                    matplotlib_version=matplotlib.__version__,
                    conditions=dict(elements_per_vector=256, beats_per_burst=16,
                                    first_beat_latency_bus_cycles=100), measurements=rows,
                    utilization_ratio=gain,
                    scope="Simulation-based operand delivery; not full-system speedup, PE activity or physical PPA.",
                    svg_sha256=hashlib.sha256(output.read_bytes()).hexdigest())
    (args.output / "mac_feed_utilization.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print("Rendered accepted utilization counts to " + output.name)


if __name__ == "__main__":
    main()
