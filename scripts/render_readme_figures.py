#!/usr/bin/env python3
"""Regenerate the README SVGs with matplotlib; optionally render PNG previews.

Architecture: rtl/mac_accel_dma_rob_top.v, rtl/mac_dma_rob.v.
ROB illustration: rtl/axi_read_engine_rob.v (illustrative, not a recorded trace).
Timing and power data: published metric tables in syn/README.md.
"""

import argparse
from pathlib import Path
import re

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "figures"
INK = "#182c43"
MUTED = "#52677e"
BLUE = "#2563a7"
TEAL = "#087e83"
PURPLE = "#7854b5"
LINE = "#cbd7e3"
BASE = "#72849a"
COLORS = [BLUE, TEAL, PURPLE]
PALE = ["#eaf2fb", "#e8f5f3", "#f1ecfa"]

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "font.size": 12,
    "text.color": INK,
    "svg.fonttype": "none",
    "svg.hashsalt": "vector-mac-readme",
    "axes.spines.top": False,
    "axes.spines.right": False,
})


def canvas(height, title, subtitle):
    fig = plt.figure(figsize=(14, height / 100), facecolor="white")
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set(xlim=(0, 1400), ylim=(height, 0))
    ax.axis("off")
    label(ax, 42, 44, title, size=23, weight="bold")
    label(ax, 42, 78, subtitle, size=11, color=MUTED)
    return fig, ax


def label(ax, x, y, text, size=12, color=INK, weight="normal", ha="left"):
    return ax.text(x, y, text, fontsize=size, color=color, weight=weight,
                   ha=ha, va="center", linespacing=1.5)


def box(ax, x, y, w, h, title="", detail="", fill="white", edge=LINE,
        title_size=13, detail_size=10):
    ax.add_patch(FancyBboxPatch((x, y), w, h,
                 boxstyle="round,pad=0,rounding_size=12",
                 linewidth=1.3, edgecolor=edge, facecolor=fill))
    if title:
        label(ax, x + w / 2, y + h / 2 - (14 if detail else 0), title,
              size=title_size, weight="bold", ha="center")
    if detail:
        label(ax, x + w / 2, y + h / 2 + 17, detail,
              size=detail_size, color=MUTED, ha="center")


def arrow(ax, points, color=BLUE, dashed=False):
    style = "--" if dashed else "-"
    if len(points) > 2:
        ax.plot(*zip(*points[:-1]), color=color, linewidth=1.7, linestyle=style)
    ax.add_patch(FancyArrowPatch(points[-2], points[-1], arrowstyle="-|>",
                 mutation_scale=12, linewidth=1.7, color=color, linestyle=style))


def architecture():
    fig, ax = canvas(715, "Vector MAC Accelerator | System Architecture",
                     "V3: mac_accel_dma_rob_top  •  MAX_OUTSTANDING = 8")
    box(ax, 65, 105, 235, 65, "Host / CPU", "AXI4-Lite control")
    box(ax, 375, 105, 300, 65, "AXI Memory", "Behavioral model in V3 simulation")
    box(ax, 20, 215, 970, 450, fill="#f0f7ff", edge="#95caff")
    box(ax, 1000, 215, 385, 450, fill="#ecfaf6", edge="#9be0cc")
    label(ax, 42, 242, "BUS CLOCK DOMAIN", 14, BLUE, "bold")
    label(ax, 1020, 242, "MAC CLOCK DOMAIN", 14, TEAL, "bold")
    label(ax, 42, 268, "AXI-Lite + DMA / ROB", 10, BLUE)
    label(ax, 1020, 268, "Independent asynchronous clock", 10, TEAL)
    box(ax, 55, 315, 235, 85, "CSR / Control", "Addresses, length, status")
    box(ax, 380, 315, 280, 85, "AXI Read DMA + ROB",
        "Up to 8 outstanding bursts;\nin-order retirement", fill=PALE[0], edge=BLUE,
        detail_size=9)
    box(ax, 380, 455, 225, 80, "Operand A Buffer", "256 x 16 bits", fill=PALE[0], edge=BLUE)
    box(ax, 660, 455, 175, 80, "Operand Pairing", "{last, a, b}", fill=PALE[0], edge=BLUE)
    box(ax, 890, 445, 210, 105, fill="#eaf7f4", edge=TEAL)
    label(ax, 995, 465, "Async FIFO", 13, weight="bold", ha="center")
    label(ax, 995, 491, "Gray pointers + 2-FF", 10, MUTED, ha="center")
    ax.plot([995, 995], [513, 541], color="#b7d9d1", linestyle=":", linewidth=1)
    label(ax, 940, 527, "Write: bus_clk", 8, color=BLUE, ha="center")
    label(ax, 1050, 527, "Read: mac_clk", 8, color=TEAL, ha="center")
    box(ax, 1140, 455, 215, 80, "MAC Pipeline", "Signed multiply / accumulate",
        fill=PALE[1], edge=TEAL, detail_size=9)

    arrow(ax, [(300, 138), (335, 138), (335, 300), (225, 300), (225, 315)])
    label(ax, 291, 183, "AXI4-Lite", 10, BLUE, "bold", ha="center")
    label(ax, 291, 201, "control", 10, BLUE, "bold", ha="center")
    arrow(ax, [(290, 358), (380, 358)])
    label(ax, 336, 378, "Command", 9, BLUE, ha="center")
    arrow(ax, [(480, 315), (480, 170)])
    arrow(ax, [(560, 170), (560, 315)])
    label(ax, 444, 188, "AR", 10, BLUE, "bold", ha="right")
    label(ax, 444, 207, "(Read address)", 9, BLUE, ha="right")
    label(ax, 575, 188, "R", 10, BLUE, "bold")
    label(ax, 575, 207, "(Read data + RID)", 9, BLUE)
    arrow(ax, [(490, 400), (490, 455)])
    label(ax, 482, 433, "Load A", 10, BLUE, "bold", ha="right")
    arrow(ax, [(660, 358), (748, 358), (748, 455)])
    label(ax, 762, 430, "Stream B", 10, BLUE, "bold")
    arrow(ax, [(605, 495), (660, 495)])
    arrow(ax, [(835, 495), (890, 495)])
    arrow(ax, [(1100, 495), (1140, 495)], TEAL)

    arrow(ax, [(180, 315), (180, 285), (1245, 285), (1245, 455)], PURPLE, True)
    label(ax, 785, 276, "Start: toggle + 2-FF + edge detect", 10, PURPLE, ha="center")
    for src_x, lane_y, dst_x in [(1190, 575, 110), (1245, 610, 150), (1300, 645, 190)]:
        arrow(ax, [(src_x, 535), (src_x, lane_y), (dst_x, lane_y),
                   (dst_x, 400)], TEAL, True)
    label(ax, 740, 563, "Done: toggle synchronization", 10, TEAL, ha="center")
    label(ax, 740, 598, "Busy: 2-FF synchronization", 10, TEAL, ha="center")
    label(ax, 740, 633, "Result / latency: stable capture on done", 10, TEAL, ha="center")
    label(ax, 42, 691, "Operand schedule: Load A first, then stream B and pair with stored A before crossing the FIFO into the MAC.", 11, BLUE, "bold")
    return fig


def rob():
    fig, ax = canvas(870, "Reorder Buffer | Receive by RID, Retire in Allocation Order",
                     "Illustrative sequence: 3 bursts, 2 beats each. Protocol schematic, not a recorded waveform.")
    for y in [205, 475, 665]:
        ax.plot([42, 1358], [y, y], color=LINE, linewidth=0.8)
    label(ax, 42, 143, "01  ISSUE", 13, BLUE, "bold")
    label(ax, 42, 172, "ARID = alloc_ptr", 10, MUTED)
    for i, x in enumerate([280, 590, 900]):
        box(ax, x, 120, 250, 65, f"Burst {i} / ARID {i}", "2 beats allocated", PALE[i], COLORS[i])
        if i < 2:
            arrow(ax, [(x + 250, 152), (x + 302, 152)], MUTED)
    label(ax, 42, 260, "02  RECEIVE", 13, TEAL, "bold")
    label(ax, 42, 291, "Interleaved R beats", 10, MUTED)
    arrivals = [(1, 0), (2, 0), (1, 1), (0, 0), (2, 1), (0, 1)]
    for j, (rid, beat) in enumerate(arrivals):
        box(ax, 280 + j * 177, 235, 155, 70, f"RID {rid} / beat {beat}",
            "RLAST" if beat == 1 else "", PALE[rid], COLORS[rid], title_size=11)
    arrow(ax, [(285, 339), (1320, 339)], MUTED)
    label(ax, 803, 358, "Time / acceptance order", 10, MUTED, ha="center")
    label(ax, 280, 388, "Completion order: burst 1, then 2, then 0. Beat order within each ID is preserved.", 10)
    box(ax, 280, 411, 565, 47, "Burst 1 completes first, but waits for burst 0 to retire.",
        fill="#fff7d7", edge="#f4c431", title_size=12)
    label(ax, 42, 523, "03  STORE", 13, PURPLE, "bold")
    label(ax, 42, 552, "RID selects entry", 10, MUTED)
    for i, x in enumerate([280, 590, 900]):
        box(ax, x, 495, 250, 105, fill=PALE[i], edge=COLORS[i])
        label(ax, x + 125, 522, f"ROB entry {i}", 13, weight="bold", ha="center")
        label(ax, x + 125, 550, f"B{i} beat 0 | B{i} beat 1", 11, MUTED, ha="center")
        label(ax, x + 125, 575, "complete = 1", 11, MUTED, ha="center")
    label(ax, 280, 629, "Snapshot after all beats arrive. Only a completed head entry can drive retirement.", 10, MUTED)
    label(ax, 42, 723, "04  RETIRE", 13, BLUE, "bold")
    label(ax, 42, 753, "head_ptr: 0 → 1 → 2", 10, MUTED)
    for j in range(6):
        rid, beat = divmod(j, 2)
        box(ax, 280 + j * 177, 695, 155, 60, f"B{rid} beat {beat}", fill=PALE[rid],
            edge=COLORS[rid], title_size=12)
    label(ax, 280, 798, "In-order output: burst 0, burst 1, burst 2. Credits return after each entry drains.", 11)
    label(ax, 42, 840, "Implementation: circular alloc/head pointers, per-entry receive counts, completion flags, and RID-indexed beat storage.", 10, MUTED)
    return fig


def metric(name):
    """Read the first published numeric comparison for an exact table-row name."""
    source = (ROOT / "syn" / "README.md").read_text()
    for row in source.splitlines():
        cells = [c.strip() for c in row.split("|")]
        if len(cells) >= 4 and cells[1] == name:
            return [float(re.search(r"[+-]?\d+(?:\.\d+)?", c).group()) for c in cells[2:4]]
    raise ValueError(f"Published metric missing: {name}")


def bar_axis(fig, rect, values, names, title, unit, upper, colors):
    ax = fig.add_axes(rect)
    ax.bar(names, values, color=colors, width=0.55, zorder=3)
    ax.set_ylim(0, upper)
    ax.set_title(title, loc="left", fontsize=13, weight="bold", pad=16, color=INK)
    ax.set_ylabel(unit, fontsize=10, color=MUTED)
    ax.tick_params(axis="both", labelsize=10, colors=MUTED, length=0)
    ax.spines["left"].set_visible(False)
    ax.spines["bottom"].set_color(LINE)
    ax.grid(axis="y", color="#e7edf3", zorder=0)
    for x, v in enumerate(values):
        ax.text(x, v + upper * 0.025, f"{v:.2f}" if unit != "levels" else f"{v:.0f}",
                ha="center", fontsize=13, weight="bold", color=INK)
    return ax


def timing():
    delay = metric("QoR critical-path length")
    levels = metric("Logic levels")
    slack = metric("Slack")
    fig, ax = canvas(850, "Timing optimization | move address arithmetic off the AR output",
                     "Design Compiler pre-layout block benchmark: mac_dma_rob  /  depth 8  /  10 ns target  /  OSU 0.18 um")
    box(ax, 42, 118, 635, 155, fill="#f4f6f9")
    box(ax, 703, 118, 655, 155, fill=PALE[0], edge="#b9cee5")
    label(ax, 65, 143, "BEFORE  /  COMBINATIONAL ADDRESS", 11, MUTED, "bold")
    label(ax, 725, 143, "AFTER  /  HANDSHAKE-DRIVEN ADDRESS STATE", 11, BLUE, "bold")
    label(ax, 65, 187, "cmd_addr_r + (issue_elem << 2)  ->  ARADDR", 12, weight="bold")
    label(ax, 65, 233, "AR output includes a long address carry chain.", 11, MUTED)
    label(ax, 725, 183, "current_ar_addr_r  ->  ARADDR", 13, weight="bold")
    label(ax, 725, 220, "On ARVALID && ARREADY: addr += burst_bytes", 11, BLUE)
    label(ax, 725, 250, "Address stays stable while the AR channel is stalled.", 10, MUTED)
    bar_axis(fig, [0.075, 0.245, 0.38, 0.32], delay, ["Original", "Optimized"],
             "Global QoR critical-path length", "ns", 6.8, [BASE, BLUE])
    bar_axis(fig, [0.565, 0.245, 0.34, 0.32], levels, ["Original", "Optimized"],
             "Global worst-path logic depth", "levels", 38, [BASE, BLUE])
    label(ax, 65, 310, f"{(1 - delay[1] / delay[0]) * 100:.2f}% shorter QoR critical path", 17, BLUE, "bold")
    label(ax, 792, 310, f"{int(levels[0])} -> {int(levels[1])} logic levels", 17, BLUE, "bold")
    box(ax, 42, 688, 1316, 69, fill="#f4f8fd")
    label(ax, 66, 722, f"Setup slack @ 100 MHz: +{slack[0]:.4f} ns  ->  +{slack[1]:.4f} ns     |     ARADDR leaves the global top 10", 13, BLUE, "bold")
    label(ax, 42, 789, "The global worst path changes from ARADDR to reset/control. These bars compare overall QoR, not the same path.", 10, MUTED)
    label(ax, 42, 821, "Source: syn/README.md, first synthesis-driven optimization. Ideal clocks; no extracted parasitics or physical closure.", 10, MUTED)
    return fig


def power():
    active = metric("Active dynamic power")
    idle = metric("Idle dynamic power")
    energy = metric("Energy per measured job")
    fig, ax = canvas(890, "Targeted clock gating | reduce switching in the A-operand buffer",
                     "SAIF-driven pre-layout block power: mac_dma_rob  /  depth 8  /  100 MHz  /  identical activity files")
    box(ax, 42, 118, 1316, 139, fill="#f1f9f7", edge="#b7d9d1")
    label(ax, 65, 145, "GATING SCOPE", 11, TEAL, "bold")
    box(ax, 65, 175, 230, 55, "buf_a only", "4096 register bits", fill="white", edge=TEAL, title_size=12, detail_size=9)
    arrow(ax, [(295, 202), (355, 202)], TEAL)
    box(ax, 355, 175, 310, 55, "256 independent banks", "16 bits per bank", fill="white", edge=TEAL, title_size=12, detail_size=9)
    label(ax, 720, 183, "Per bank: discrete LATCH + AND clock gate", 12, weight="bold")
    label(ax, 720, 218, "ROB, control, FIFO and synchronizers are excluded.", 11, MUTED)
    bar_axis(fig, [0.08, 0.26, 0.36, 0.32], active, ["Baseline", "Gated"],
             "Active dynamic power", "mW", 95, [BASE, TEAL])
    bar_axis(fig, [0.565, 0.26, 0.36, 0.32], idle, ["Baseline", "Gated"],
             "Idle dynamic power", "mW", 95, [BASE, TEAL])
    label(ax, 112, 312, f"-{(1 - active[1] / active[0]) * 100:.2f}%", 24, TEAL, "bold")
    label(ax, 792, 312, f"-{(1 - idle[1] / idle[0]) * 100:.2f}%", 24, TEAL, "bold")
    box(ax, 42, 722, 1316, 69, fill="#f1f9f7")
    label(ax, 65, 756, f"Energy per measured job: {energy[0]:.3f} nJ  ->  {energy[1]:.3f} nJ    |    {(1 - energy[1] / energy[0]) * 100:.2f}% reduction", 14, TEAL, "bold")
    label(ax, 42, 826, "Same deterministic workload and active/idle SAIF windows; all 4096 buf_a bits annotated in both designs.", 10, MUTED)
    label(ax, 42, 859, "Source: syn/README.md, targeted buf_a experiment. Discrete latch/logic gates; no production ICG, CTS or post-layout power.", 10, MUTED)
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preview-dir", type=Path, help="Optional PNG preview destination")
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    if args.preview_dir:
        args.preview_dir.mkdir(parents=True, exist_ok=True)
    for name, draw in [("architecture", architecture), ("rob_reordering", rob),
                       ("timing_optimization", timing), ("power_comparison", power)]:
        fig = draw()
        svg_path = OUT / f"{name}.svg"
        fig.savefig(svg_path, metadata={"Date": None,
                    "Title": name.replace("_", " "),
                    "Description": "Vector MAC accelerator README figure; sources documented in scripts/render_readme_figures.py"})
        # Matplotlib leaves spaces before newlines inside SVG path attributes.
        svg_path.write_text("\n".join(line.rstrip() for line in svg_path.read_text().splitlines()) + "\n")
        if args.preview_dir:
            fig.savefig(args.preview_dir / f"{name}.png", dpi=120)
        plt.close(fig)
        print(f"Generated docs/figures/{name}.svg")


if __name__ == "__main__":
    main()
