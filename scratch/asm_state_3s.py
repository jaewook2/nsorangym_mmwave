#!/usr/bin/env python3
"""ns-3 check: two mmWave cells cover every ASM transition, 3 s apart.

Modes are 1 Active, 2 ASM1, 3 ASM2, 4 ASM3, 5 ASM4, 6 OFF.
The 30 ordered pairs are split into two walks. Times are the Salem/IMEC
sleep levels. Powers are the pico sleep totals, and wake power is twice
the operating power of the component being turned on.
"""
import argparse
import csv
import os
import subprocess
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
BINARY = os.path.join(ROOT, "build/scratch/ns3.38.rc1-scenario-asm-verify-default")
MODES = (1, 2, 3, 4, 5, 6)
NAMES = {1: "Active", 2: "ASM1", 3: "ASM2", 4: "ASM3", 5: "ASM4", 6: "OFF"}
PHASES = {0: "Stable", 1: "Deactivating", 2: "Waking"}
CELL_LEFT = 2
CELL_RIGHT = 3
STEP_MS = 3000

# Seconds. Salem et al., IEEE NCA 2019, Table I. On equals off.
OFF_TIMES = {
    "asmPaOffTime": 35.5e-6,
    "asmRfOffTime": 0.5e-3,
    "asmBbOffTime": 5e-3,
    "asmControlToMinimalTime": 0.5,
    "asmMinimalOffTime": 0.5,
}
ON_TIMES = {
    "asmOffToMinimalTime": 0.5,
    "asmMinimalToControlTime": 0.5,
    "asmBbOnTime": 5e-3,
    "asmRfOnTime": 0.5e-3,
    "asmPaOnTime": 35.5e-6,
}
# Pico watts. Control is the SM3 total and Minimal the SM4 total.
# Wake is 200% of that component. PA wake is twice the 1 W TxPower reference.
POWERS = {
    "asmRfPower": 1.142,
    "asmBbPower": 0.067,
    "asmControlPower": 0.301,
    "asmMinimalPower": 0.238,
    "asmPaWakePower": 2.0,
    "asmRfWakePower": 2.284,
    "asmBbWakePower": 0.134,
    "asmControlWakePower": 0.602,
    "asmMinimalWakePower": 0.476,
}
OFF_S = (
    OFF_TIMES["asmPaOffTime"],
    OFF_TIMES["asmRfOffTime"],
    OFF_TIMES["asmBbOffTime"],
    OFF_TIMES["asmControlToMinimalTime"],
    OFF_TIMES["asmMinimalOffTime"],
)
ON_S = (
    ON_TIMES["asmPaOnTime"],
    ON_TIMES["asmRfOnTime"],
    ON_TIMES["asmBbOnTime"],
    ON_TIMES["asmMinimalToControlTime"],
    ON_TIMES["asmOffToMinimalTime"],
)


def all_edges():
    return [(src, dst) for src in MODES for dst in MODES if src != dst]


def eulerian_edges():
    pending = {src: [dst for dst in MODES if dst != src] for src in MODES}
    stack = [MODES[0]]
    vertices = []
    while stack:
        src = stack[-1]
        if pending[src]:
            stack.append(pending[src].pop())
        else:
            vertices.append(stack.pop())
    vertices.reverse()
    edges = list(zip(vertices[:-1], vertices[1:]))
    if len(edges) != 30 or set(edges) != set(all_edges()):
        raise RuntimeError(f"walk does not cover every transition: {len(edges)}")
    return edges


def two_walks():
    edges = eulerian_edges()
    starts = [index for index, (src, _dst) in enumerate(edges) if src == 1]
    rotate = starts[0]
    edges = edges[rotate:] + edges[:rotate]
    starts = [index for index, (src, _dst) in enumerate(edges) if src == 1 and index != 0]
    split = min(starts, key=lambda index: abs(index - len(edges) / 2))
    left = edges[:split]
    right = edges[split:]
    for walk in (left, right):
        if walk[0][0] != 1:
            raise RuntimeError("walk does not start at Active")
        for prev, nxt in zip(walk, walk[1:]):
            if prev[1] != nxt[0]:
                raise RuntimeError("walk is not continuous")
    if set(left + right) != set(all_edges()):
        raise RuntimeError("the two walks do not cover every transition")
    return left, right


def write_csv(path: str, left, right) -> None:
    rows = []
    for index, (_src, dst) in enumerate(left):
        rows.append((STEP_MS * (index + 1), CELL_LEFT, dst))
    for index, (_src, dst) in enumerate(right):
        rows.append((STEP_MS * (index + 1), CELL_RIGHT, dst))
    rows.sort()
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["timestampMs", "cellId", "asmMode"])
        writer.writerows(rows)


def expected_delay_s(src: int, dst: int) -> float:
    src_depth = src - 1
    dst_depth = dst - 1
    if dst_depth > src_depth:
        return sum(OFF_S[src_depth:dst_depth])
    return sum(ON_S[dst_depth:src_depth])


def sim_time(left, right) -> float:
    steps = max(len(left), len(right))
    return steps * STEP_MS / 1000.0 + 2.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        default=os.path.join(os.path.dirname(__file__), "asm-state-3s-out"),
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    left, right = two_walks()
    duration = sim_time(left, right)
    print(f"cell {CELL_LEFT}: {len(left)} transitions, cell {CELL_RIGHT}: {len(right)}")
    print("off times s: " + ", ".join(f"{key}={value}" for key, value in OFF_TIMES.items()))
    print(f"simTime={duration:.1f} s, step=3 s")
    if args.dry_run:
        for cell, walk in ((CELL_LEFT, left), (CELL_RIGHT, right)):
            chain = " -> ".join(NAMES[src] for src, _dst in walk) + " -> " + NAMES[walk[-1][1]]
            print(f"  cell {cell}: {chain}")
        return 0

    os.makedirs(args.output, exist_ok=True)
    csv_path = os.path.join(args.output, "asm_actions_for_ns3.csv")
    write_csv(csv_path, left, right)
    if not os.path.isfile(BINARY):
        print(f"missing {BINARY}", file=sys.stderr)
        return 1

    command = [
        BINARY,
        f"--simTime={duration}",
        "--ues=8",
        "--nMmWaveEnb=2",
        "--asmStateMachine=true",
        "--tputPeriod=0.5",
        f"--controlFileNameAsm={csv_path}",
        "--scheduleAsmControl=true",
        "--useSemaphores=false",
    ]
    for key, value in {**OFF_TIMES, **ON_TIMES, **POWERS}.items():
        command.append(f"--{key}={value}")
    subprocess.run(command, cwd=args.output, check=True)

    changes = []
    with open(os.path.join(args.output, "asm-mode-changes.txt")) as handle:
        for row in csv.DictReader(handle):
            changes.append(row)
    tput = []
    with open(os.path.join(args.output, "asm-path-tput.csv")) as handle:
        tput = list(csv.DictReader(handle))
    ue_rows = []
    ue_path = os.path.join(args.output, "ue-app-sat.csv")
    if os.path.isfile(ue_path):
        with open(ue_path) as handle:
            ue_rows = list(csv.DictReader(handle))

    def pair_map(text, width):
        parsed = {}
        for part in (text or "").split(";"):
            bits = part.split(":")
            if len(bits) >= width:
                parsed[int(bits[0])] = bits[1:]
        return parsed

    def ue_mbps(time_ms, cell):
        rates = []
        for row in ue_rows:
            if int(float(row["timeMs"])) != time_ms:
                continue
            if int(float(row.get("servingNrCellId") or 0)) != cell:
                continue
            if int(float(row.get("useMmWave") or 0)) != 1:
                continue
            rates.append(float(row.get("rxKbps") or 0) / 1000.0)
        if not rates:
            return 0.0, 0
        return sum(rates) / len(rates), len(rates)

    print(
        "\ncell  transition        done_ms  delay_ms  expect_ms  "
        "state     dataSym  ueMbps  nUe  cellW"
    )
    bad = []
    seen = set()
    for row in changes:
        cell = int(row["cellId"])
        old = int(row["oldMode"])
        new = int(row["newMode"])
        done = int(row["timeMs"])
        symbols = int(row["dataSymbols"])
        seen.add((cell, old, new))
        expect_ms = expected_delay_s(old, new) * 1000.0
        command = None
        for mark in range(STEP_MS, done + STEP_MS, STEP_MS):
            if mark <= done and abs((done - mark) - expect_ms) <= 1.0:
                command = mark
                break
        delay = None if command is None else done - command
        after = next((item for item in tput if int(item["timeMs"]) >= done), None)
        phase = "?"
        cell_w = float("nan")
        ue_rate, n_ue = 0.0, 0
        if after is not None:
            state = pair_map(after.get("asmState"), 2).get(cell, ["?"])
            phase = PHASES.get(int(state[0]), state[0]) if state[0].isdigit() else state[0]
            power = pair_map(after.get("cellPower"), 1).get(cell)
            if power:
                cell_w = float(power[0])
            ue_rate, n_ue = ue_mbps(int(after["timeMs"]), cell)
        label = f"{NAMES[old]}->{NAMES[new]}"
        delay_s = "" if delay is None else f"{delay:8d}"
        print(
            f"{cell:<4}  {label:<16}  {done:7d}  {delay_s:>8}  {expect_ms:9.3f}  "
            f"{phase:<9}  {symbols:7d}  {ue_rate:6.2f}  {n_ue:3d}  {cell_w:6.3f}"
        )
        if symbols != (12 if new == 1 else 0):
            bad.append(f"cell {cell} {label} dataSymbols={symbols}")
        if phase != "Stable":
            bad.append(f"cell {cell} {label} phase={phase}")
        if delay is None or abs(delay - expect_ms) > 1.0:
            bad.append(f"cell {cell} {label} delay={delay} expect={expect_ms:.3f}")

    covered = {(old, new) for _cell, old, new in seen}
    missing = set(all_edges()) - covered
    print(f"\ncovered {len(covered)}/30 transitions")
    if missing:
        print(f"missing {sorted(missing)}")
    if bad:
        print("mismatches:")
        for item in bad:
            print(f"  {item}")
    print(f"\nwrote {args.output}")
    return 0 if not missing and not bad else 1


if __name__ == "__main__":
    raise SystemExit(main())
