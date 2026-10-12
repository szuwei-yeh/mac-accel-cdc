#!/usr/bin/env python3
"""Dependency-free checks for published documentation and preserved inputs."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def files():
    output = subprocess.check_output(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
        cwd=ROOT)
    return sorted({ROOT / name.decode() for name in output.split(b"\0") if name})


def prose(text):
    lines = []
    fence = None
    for line in text.splitlines():
        match = re.match(r"^\s*(`{3,}|~{3,})", line)
        if match:
            mark = match[1]
            if fence is None:
                fence = mark
            elif mark[0] == fence[0] and len(mark) >= len(fence):
                fence = None
        elif fence is None:
            lines.append(line)
    if fence:
        raise ValueError("Unclosed Markdown code fence")
    return "\n".join(lines)


def anchors(text):
    ids = set(re.findall(r'<a\s+id="([^"]+)"', text))
    counts = {}
    for heading in re.findall(r"^#{1,6}\s+(.+?)\s*#*\s*$", prose(text), re.M):
        # GitHub heading fragments retain Unicode letters and remove punctuation.
        heading = re.sub(r"!?\[([^]]+)\]\([^)]*\)", r"\1", heading)
        slug = re.sub(r"[^\w\- ]", "", heading.lower()).replace(" ", "-")
        count = counts.get(slug, 0)
        counts[slug] = count + 1
        ids.add(slug + (f"-{count}" if count else ""))
    return ids


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--python-syntax", action="store_true")
    args = parser.parse_args()
    published = files()
    if args.python_syntax:
        sources = [p for p in published if p.suffix == ".py"]
        for path in sources:
            compile(path.read_bytes(), str(path.relative_to(ROOT)), "exec")
        print(f"PYTHON_SYNTAX_PASS {len(sources)} files (no EDA execution)")
        return 0

    errors = []
    links = 0
    for path in published:
        if path.suffix in {".md", ".py", ".v", ".sv", ".sby", ".json", ".tcl",
                           ".mk", ".sh", ".csv", ".yml", ".yaml", ".svg"}:
            text = path.read_text()
            patterns = [r"/(?:Users|home|fs/student)/[^\s\"']+",
                        r"\b(?:ghp_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{30,})",
                        r"-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----"]
            if any(re.search(pattern, text) for pattern in patterns):
                errors.append(f"{path.relative_to(ROOT)}: private path/credential pattern")
        if path.suffix == ".md":
            try:
                body = prose(path.read_text())
                for target in re.findall(r"!?\[[^\]\n]+\]\(([^)\n]+)\)", body):
                    target = target.strip().split(' "', 1)[0].strip("<>")
                    parsed = urlsplit(target)
                    if parsed.scheme or parsed.netloc:
                        continue
                    links += 1
                    dest = (path.parent / unquote(parsed.path)).resolve() if parsed.path else path
                    if not dest.is_relative_to(ROOT) or not dest.exists():
                        errors.append(f"{path.relative_to(ROOT)}: missing local link {target}")
                    elif parsed.fragment and dest.suffix == ".md":
                        if unquote(parsed.fragment) not in anchors(dest.read_text()):
                            errors.append(f"{path.relative_to(ROOT)}: missing fragment {target}")
            except ValueError as error:
                errors.append(f"{path.relative_to(ROOT)}: {error}")
        if path.suffix == ".png" and path.read_bytes()[:8] != b"\x89PNG\r\n\x1a\n":
            errors.append(f"{path.relative_to(ROOT)}: invalid PNG signature")

    pins = json.loads((ROOT / "physical/pins.json").read_text())
    from revision_evidence import validate_evidence
    try:
        validate_evidence(ROOT, pins)
    except (ValueError, KeyError, OSError, StopIteration) as error:
        errors.append("Revision evidence validation failed: " + str(error))
    for name, record in pins["template_provenance"].items():
        if digest(ROOT / "physical/templates" / name) != record["template_sha256"]:
            errors.append("Pinned template hash changed: " + name)
    manifest = json.loads((ROOT / "physical/figures/p9_provenance.json").read_text())
    summary = json.loads((ROOT / "physical/summary.json").read_text())
    for kind, expected in manifest["accepted_artifact_sha256"].items():
        if expected != summary["artifacts"][kind]["sha256"]:
            errors.append("P9 accepted artifact reference mismatch: " + kind)
    for name, record in manifest["figures"].items():
        path = ROOT / "physical/figures" / name
        if digest(path) != record["sha256"] or path.stat().st_size != record["bytes"]:
            errors.append("P9 figure hash/size changed: " + name)
    if errors:
        print("\n".join(errors))
        return 1
    print(f"PORTFOLIO_PASS {links} local links; source/template/P9 hashes; PNGs; basic privacy scan")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
