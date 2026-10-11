#!/usr/bin/env python3
"""Render presentation-only figures without changing original evidence.

Charts read accepted public JSON/tables; layout SVGs retain the original raster body,
axes and legend with new outer labels. No GDS/ODB/STA/EDA tool is executed.
"""

import argparse
import base64
import hashlib
import json
import re
from pathlib import Path
from xml.sax.saxutils import escape

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
STAGES = ["Post-Synthesis", "Post-Placement", "Post-CTS", "Post-Route"]
TICKS = ["Post-\nSynthesis", "Post-\nPlacement", "Post-CTS", "Post-Route"]
MODELS = ["Pre-layout / ideal clocks", "Estimated RC / ideal clocks",
          "Estimated RC / propagated clocks", "Extracted RC / propagated clocks"]
BLUE, PURPLE = "#247da8", "#8c5ba5"


def sha(data):
    return hashlib.sha256(data).hexdigest()


def read(relative):
    return (ROOT / relative).read_bytes()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True,
                        help="New, nonexistent presentation output directory")
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    summary = json.loads(read("physical/summary.json"))
    hold = json.loads(read("physical/hold-analysis.json"))
    rows = summary["stages"]
    if [r["stage"] for r in rows] != ["P2 synthesis", "P3 placement", "P3 CTS", "P4 routed"]:
        raise ValueError("Unexpected historical checkpoint order")
    sources = ["physical/summary.json", "physical/hold-analysis.json",
               "physical/figures/provenance.json", "physical/figures/p9_provenance.json",
               "docs/figures/power_comparison.svg", "syn/README.md",
               "rtl/mac_dma_rob.v",
               "scripts/render_presentation_figures.py"]
    manifest = {
        "scope": "Presentation only; accepted measurements and original images unchanged. No EDA execution.",
        "renderer": {"matplotlib": matplotlib.__version__, "numpy": np.__version__},
        "stage_labels": dict(zip([r["stage"] for r in rows], STAGES)),
        "timing_models": dict(zip(STAGES, MODELS)),
        "inputs_sha256": {s: sha(read(s)) for s in sources},
        "accepted_artifact_sha256": {k: summary["artifacts"][k]["sha256"] for k in ["gds", "odb"]},
        "figures": {},
    }
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 13,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "svg.fonttype": "none", "svg.hashsalt": "vector-mac-engineering-stages"})

    def record(name, data, details):
        if name.endswith(".svg") and "preserved_body_pixels" not in details:
            # The pinned plotting font may not be installed on a reader's Mac.
            # Keep geometry/numeric text unchanged and specify common fallbacks.
            text = data.decode().replace("</svg>",
                "<style>text{font-family:Arial,Helvetica,sans-serif !important}</style>\n</svg>")
            data = text.encode()
        (out / name).write_bytes(data)
        directory = "docs/figures/" if name == "power-gating-presentation.svg" else "physical/figures/"
        manifest["figures"][name] = {"sha256": sha(data), "repository_path": directory + name, **details}

    def save(fig, name, caption, values, source):
        fig.text(.12, .012, caption, fontsize=10, color="#475569", va="bottom")
        from io import BytesIO
        buffer = BytesIO()
        fig.savefig(buffer, format="svg", metadata={"Date": None, "Title": name,
                    "Description": caption})
        data = ("\n".join(line.rstrip() for line in buffer.getvalue().decode().splitlines()) + "\n").encode()
        record(name, data, {"method": "Replot unchanged accepted data", "source": source,
                           "caption": caption, "plotted_values": values})
        plt.close(fig)

    x = np.arange(4)
    fig, axes = plt.subplots(2, 1, figsize=(7, 8.4))
    fig.subplots_adjust(left=.14, right=.96, top=.94, bottom=.22, hspace=.65)
    values = {}
    for ax, mode, scale, title, unit in zip(axes, ["max", "min"], [1, 1000],
                                           ["Setup margin", "Hold margin"], ["Worst slack (ns)", "Worst slack (ps)"]):
        for clock, color in [("bus_clk", BLUE), ("mac_clk", PURPLE)]:
            y = [r["timing"][clock][mode]["wns_ns"] * scale for r in rows]
            values[f"{clock}_{mode}"] = y
            ax.plot(x, y, "o-", color=color, label=clock.replace("_", " "))
            for i, v in enumerate(y):
                offset = 10
                if mode == "min" and clock == "bus_clk" and i != 2:
                    offset = -22
                ax.annotate(f"{v:.3f}", (i, v), xytext=(0, offset), textcoords="offset points",
                            ha="center", fontsize=11, color=color)
        ax.set_xticks(x, TICKS, fontsize=11)
        ax.set_title(title, loc="left", fontweight="bold", pad=12)
        ax.set_ylabel(unit)
        ax.grid(axis="y", alpha=.18)
        ax.legend(fontsize=10, loc="lower left" if mode == "min" else "upper right")
        ax.set_xlim(-.18, 3.18)
    axes[0].set_ylim(5.72, 8.65)
    axes[1].set_ylim(-4, 97)
    axes[1].axhline(0, color="#555", lw=.6)
    save(fig, "timing-stages-presentation.svg",
         "Bus 10 ns / MAC 7.5 ns; all TNS = 0 ns.\n"
         "Synthesis: pre-layout / ideal clocks. Placement: estimated RC / ideal clocks.\n"
         "CTS: estimated RC / propagated clocks. Route: OpenRCX / propagated clocks.", values,
         "physical/summary.json: stages[*].timing; ns setup, ps hold")

    fig, axes = plt.subplots(2, 1, figsize=(7, 8.2))
    fig.subplots_adjust(left=.16, right=.96, top=.94, bottom=.17, hspace=.65)
    values = {}
    for ax, key, title, unit in [(axes[0], "standard_area_um2", "Standard-cell area", "Area (µm²)"),
                                 (axes[1], "standard_cells", "Standard-cell count", "Cells")]:
        y = [r[key] for r in rows]
        values[key] = y
        ax.bar(x, y, color=["#879eac", "#a2b4c0", "#5588a5", BLUE], width=.6)
        for i, v in enumerate(y):
            ax.text(i, v + max(y) * .025, f"{v:,.1f}" if key.endswith("um2") else f"{v:,}",
                    ha="center", fontsize=11)
        ax.set_ylim(0, max(y) * 1.19)
        ax.set_xticks(x, TICKS, fontsize=11)
        ax.set_title(title, loc="left", fontweight="bold", pad=12)
        ax.set_ylabel(unit)
        ax.grid(axis="y", alpha=.15)
    save(fig, "area-stages-presentation.svg",
         "Taps/fillers excluded. Core = 172,038.160 µm².\n"
         "Counts include functional clock buffers, CTS dummy loads and hold buffers.", values,
         "physical/summary.json: stages[*].standard_area_um2 and standard_cells")

    fig, axes = plt.subplots(2, 1, figsize=(7, 8.4))
    fig.subplots_adjust(left=.16, right=.96, top=.93, bottom=.18, hspace=.72)
    values = {}
    for ax, ident, title in zip(axes, ["p3_worst", "final_worst"],
                               ["Post-CTS worst endpoint: ROB[52][6]", "Post-Route worst endpoint: ROB[37][1]"]):
        item = next(q for q in hold["comparisons"] if q["id"] == ident)
        d = item["deltas_ns"]
        y = [1000*d["cell_delay_ns"], 1000*d["wire_delay_ns"],
             -1000*d["capture_clock_early_ns"], -1000*d["clock_to_required_delta_ns"], 1000*d["slack_ns"]]
        values[ident] = y
        ax.bar(np.arange(5), y, color=[BLUE, "#7eafc7", "#b77a48", "#b7a88d", "#57916e"])
        ax.axhline(0, color="#555", lw=.6)
        ax.set_xticks(np.arange(5), ["Cell\ndelay", "Wire\ndelay", "Capture\nclock", "Hold\narc", "Net slack\ngain"], fontsize=11)
        ax.set_title(title, loc="left", fontsize=13, fontweight="bold", pad=16)
        ax.set_ylabel("Contribution (ps)")
        ax.set_ylim(min(y) - 25, max(y) + 30)
        for i, v in enumerate(y):
            ax.annotate(f"{v:+.3f}", (i, v), xytext=(0, 6 if v >= 0 else 8 if v < -10 else -16),
                        textcoords="offset points", ha="center", fontsize=11)
    save(fig, "hold-path-contributions-presentation.svg",
         "Matched input → D-pin paths: Post-CTS to Post-Route; estimated → extracted RC.\n"
         "Capture-clock / library-hold increases subtract from hold slack.\n"
         "Different worst endpoints; observational analysis, not a same-RC experiment.", values,
         "physical/hold-analysis.json: comparisons[p3_worst,final_worst].deltas_ns × 1000")

    def metric(name):
        for line in read("syn/README.md").decode().splitlines():
            cells = [c.strip() for c in line.split("|")]
            if len(cells) >= 4 and cells[1] == name:
                return [float(re.search(r"[+-]?\d+(?:\.\d+)?", c).group()) for c in cells[2:4]]
        raise ValueError("Missing accepted power metric: " + name)

    active, idle, energy = [metric(name) for name in
                            ["Active dynamic power", "Idle dynamic power", "Estimated energy per job"]]
    fig, axes = plt.subplots(2, 1, figsize=(7, 9.5))
    fig.subplots_adjust(left=.16, right=.96, top=.73, bottom=.19, hspace=.65)
    fig.text(.16, .955, "Targeted A-buffer clock gating", fontsize=17, weight="bold")
    fig.text(.16, .925, "OSU 0.18 µm DMA block · depth 8 · 100 MHz", fontsize=12)
    fig.text(.16, .825, "buf_a: 256 × 16-bit storage entries (4,096 bits).\n"
             "Word-level discrete LATCH + AND gating; no dedicated ICG.\n"
             "ROB, control, FIFO, MAC and CDC outside the gating scope.", fontsize=11, linespacing=1.6)
    for ax, y, title in zip(axes, [active, idle], ["Active dynamic power", "Idle dynamic power"]):
        ax.bar(["Baseline", "Gated"], y, color=["#72849a", "#087e83"], width=.55)
        ax.set_ylim(0, 100)
        ax.set_ylabel("Power (mW)")
        ax.set_title(f"{title}  (−{(1-y[1]/y[0])*100:.2f}%)", loc="left", fontsize=13, weight="bold", pad=14)
        ax.grid(axis="y", alpha=.15)
        for i, v in enumerate(y):
            ax.text(i, v+3, f"{v:.4f}", ha="center", fontsize=12, weight="bold")
    save(fig, "power-gating-presentation.svg",
         f"Estimated energy/job: {energy[0]:.3f} → {energy[1]:.3f} nJ.\n"
         "Identical deterministic active/idle SAIF files; 40-cycle memory latency.\n"
         "Pre-layout estimates; no CTS, extracted power or Nangate45 power claim.",
         {"active_dynamic_mw": active, "idle_dynamic_mw": idle, "energy_per_job_nj": energy},
         "syn/README.md: accepted SAIF power table; original docs/figures/power_comparison.svg preserved")

    # Preserve the complete raster body, including axes/legend, at its native scale.
    # Only the outer header/footer strips are relabelled in a separate SVG.
    specs = [
        ("overview_dark.png", "routed-overview-presentation.svg", "Vector MAC V3 | Final routed layout",
         "Nangate45 | M3–M7 | Die: 417.39 × 417.39 µm",
         "Accepted post-route GDS • selected routing layers • full die physical coordinates",
         "mac_accel_dma_rob_top • actual accepted geometry • source rendered in KLayout 0.28.17"),
        ("routing_detail.png", "routing-detail-presentation.svg", "Vector MAC V3 | Routed-metal detail",
         "Post-Route GDS | M1–M5 and V1–V4 | 30 × 30 µm window",
         "Accepted post-route GDS • x = 195–225 µm; y = 195–225 µm • selected layers",
         "mac_accel_dma_rob_top • actual accepted geometry • source rendered in KLayout 0.28.17"),
        ("placement_overview.png", "placement-power-presentation.svg", "Vector MAC V3 | Placement and power grid",
         "Post-Route ODB | 29,175 cell footprints | Power-only M1 / M4 / M7",
         "Accepted final ODB • exact cell footprints and power shapes; fillers / taps omitted",
         "mac_accel_dma_rob_top • signal routes not displayed • no IR/EM inference"),
        ("pdn_detail.png", "power-grid-detail-presentation.svg", "Vector MAC V3 | Cells and power-grid detail",
         "Post-Route ODB | Cell footprints and power shapes | 60 × 60 µm window",
         "Accepted final ODB • x = 170–230 µm; y = 195–255 µm • power-only view",
         "mac_accel_dma_rob_top • signal routes not displayed • no IR/EM inference"),
    ]
    originals = json.loads(read("physical/figures/p9_provenance.json"))
    for source, name, title, subtitle, footer, footer2 in specs:
        relative = "physical/figures/" + source
        data = read(relative)
        pin = originals["figures"][source]
        if sha(data) != pin["sha256"]:
            raise ValueError("Original image hash mismatch: " + source)
        manifest["inputs_sha256"][relative] = sha(data)
        encoded = base64.b64encode(data).decode()
        svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="2200" height="2552" viewBox="0 0 2200 2552">
<title>{escape(title)}</title>
<desc>Presentation-only labels. Original accepted image body, axes and legend preserved without rescaling.</desc>
<rect width="2200" height="2552" fill="#0f1720"/>
<defs><clipPath id="accepted-body"><rect x="0" y="200" width="2200" height="2180"/></clipPath></defs>
<image width="2200" height="2552" clip-path="url(#accepted-body)" href="data:image/png;base64,{encoded}"/>
<g font-family="DejaVu Sans, sans-serif">
<text x="187" y="90" fill="#e6edf3" font-size="56" font-weight="bold">{escape(title)}</text>
<text x="187" y="166" fill="#a8b7c6" font-size="32">{escape(subtitle)}</text>
<text x="187" y="2420" fill="#a8b7c6" font-size="28">{escape(footer)}</text>
<text x="187" y="2480" fill="#a8b7c6" font-size="28">{escape(footer2)}</text>
</g></svg>
'''
        record(name, svg.encode(), {"method": "SVG composition of exact accepted PNG raster body with new outer text strips; no geometry re-render",
                                    "source": relative, "source_sha256": sha(data),
                                    "preserved_body_pixels": {"x": 0, "y": 200, "width": 2200, "height": 2180},
                                    "source_geometry": pin["geometry_source"], "visible_geometry": pin["visible_geometry"],
                                    "window": pin["window"], "title": title, "subtitle": subtitle,
                                    "footer": [footer, footer2]})
    (out / "presentation-provenance.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    print(f"PRESENTATION_RENDERED {len(manifest['figures'])} figures; unchanged accepted inputs")


if __name__ == "__main__":
    main()
