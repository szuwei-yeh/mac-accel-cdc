#!/usr/bin/env python3
"""Package successful synthesis + independent DC readback for standalone PT."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


PAIR = ("mac_accel_dma_rob_top_mapped.v", "mac_accel_dma_rob_top.sdc")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path):
    return json.loads(path.read_text())


def validate_run(run, stage):
    manifest = load(run / "manifest.json")
    status = load(run / "status.json")
    if (manifest.get("stage") != stage or status.get("tool_return_code") != 0
            or status.get("stage_complete") is not True or status.get("errors") != []):
        raise ValueError("Run did not complete successfully: " + str(run))
    if "WTOP_STAGE_COMPLETE=" + stage not in (run / "dc.log").read_text():
        raise ValueError("Missing completion marker: " + str(run))
    for name, expected in manifest["source_sha256"].items():
        if digest(run / "inputs" / name) != expected:
            raise ValueError("Input snapshot changed: " + name)
    if not list((run / "reports").glob("*.rpt")):
        raise ValueError("Missing reports: " + str(run))
    return manifest


def package(synthesis, readback, output):
    # Validation precedes output creation; no tool execution or input mutation.
    if output.exists():
        raise ValueError("Output already exists: " + str(output))
    sm = validate_run(synthesis, "synthesis")
    rm = validate_run(readback, "readback")
    if rm.get("parent_synthesis_manifest") != sm:
        raise ValueError("Readback belongs to a different synthesis manifest")
    if rm["library_sha256"] != sm["library_sha256"]:
        raise ValueError("DC library changed between synthesis and readback")
    hashes = {}
    for name in PAIR:
        hashes[name] = digest(synthesis / "mapped" / name)
        if (hashes[name] != rm["source_sha256"]["mapped/" + name]
                or hashes[name] != digest(readback / "inputs/mapped" / name)):
            raise ValueError("Mapped pair differs from independent readback: " + name)
    output.mkdir(parents=True, exist_ok=False)
    for src, dst in ((synthesis, output), (readback, output / "readback")):
        dst.mkdir(exist_ok=True)
        for name in ("manifest.json", "status.json", "dc.log"):
            shutil.copyfile(str(src / name), str(dst / name))
        for name in ("inputs", "reports"):
            shutil.copytree(str(src / name), str(dst / name))
    (output / "mapped").mkdir()
    for name in PAIR:
        shutil.copyfile(str(synthesis / "mapped" / name), str(output / "mapped" / name))
        if digest(output / "mapped" / name) != hashes[name]:
            raise ValueError("Mapped input changed during copy: " + name)
    # Recheck the copied evidence before writing the acceptance record last.
    copied_sm = validate_run(output, "synthesis")
    copied_rm = validate_run(output / "readback", "readback")
    if copied_sm != sm or copied_rm != rm:
        raise ValueError("Manifest changed during copy")
    acceptance = {
        "status": "MAPPED_PAIR_VERIFIED_BY_INDEPENDENT_DC_READBACK",
        "synthesis_run": str(synthesis), "readback_run": str(readback),
        "mapped_sha256": hashes,
        "note": "Execution/provenance gate only; review timing, coverage and warnings separately.",
    }
    (output / "ACCEPTANCE.json").write_text(json.dumps(acceptance, indent=2) + "\n")
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--synthesis", type=Path, required=True)
    parser.add_argument("--readback", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        result = package(args.synthesis.resolve(), args.readback.resolve(), args.output.resolve())
    except (ValueError, KeyError, OSError) as exc:
        parser.exit(1, "Bundle rejected: " + str(exc) + "\n")
    print("ACCEPTED_BUNDLE=" + str(result))


if __name__ == "__main__":
    main()
