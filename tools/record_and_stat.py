#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
record_and_stat.py (complete)
- amixer 状態表示（任意）
- （任意）安全なデフォルト設定を適用（Boost=0, Dmic0=100%, PGA=50%）
- arecord で録音（頑丈）
    - 指定chで失敗したら自動で 2ch にフォールバック（stderr を確実に捕捉）
- （任意）sox で mono 化してから評価
- sox stat
- [判定] + [Suggestion]（次に打つべき amixer コマンドをそのまま提示）

Dependencies:
  sudo apt-get install -y alsa-utils sox

推奨（あなたの環境: sof-hda-dsp / DMIC16kHz = hw:0,7）:
  ./record_and_stat.py --apply-defaults --mono

例:
  ./record_and_stat.py
  ./record_and_stat.py --device hw:0,7 --rate 16000 --channels 1 --mono --apply-defaults
  ./record_and_stat.py --device hw:0,6 --rate 48000 --channels 2

Note:
  arecord -d は環境によって整数秒のみ → sec は丸めて int で渡す
"""

from __future__ import annotations

import argparse
import shlex
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple


# ---------------- helpers ----------------

def sh(cmd: List[str]) -> str:
    return " ".join(shlex.quote(x) for x in cmd)


def amixer_sget(card: int, control: str) -> str:
    cp = subprocess.run(
        ["amixer", "-c", str(card), "sget", control],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    out = (cp.stdout or "") + (cp.stderr or "")
    return out.strip()


def amixer_sset(card: int, control: str, *values: str) -> None:
    cp = subprocess.run(
        ["amixer", "-c", str(card), "sset", control, *values],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if cp.returncode != 0:
        print(f"[WARN] amixer set failed: {control} {' '.join(values)}", file=sys.stderr)
        if cp.stderr:
            print(cp.stderr.strip(), file=sys.stderr)


CONTROLS_TO_SHOW = [
    "Input Source",
    "Capture",
    "Dmic0",
    "Dmic1 2nd",
    "PGA2.0 2 Master",
    "PGA4.0 4 Master",
    "Headset Mic Boost",
    "Headphone Mic Boost",
]


def show_controls(card: int, title: str) -> None:
    print(f"\n[{title}]")
    for c in CONTROLS_TO_SHOW:
        t = amixer_sget(card, c)
        if t:
            print(t)
            print("-" * 40)


def apply_defaults(card: int) -> None:
    """
    DMIC の安全な開始点（歪み回避優先）
      - Boost 0
      - Dmic0 cap + 100%
      - PGA 50%
      - Capture cap（害はない）
    """
    amixer_sset(card, "Headset Mic Boost", "0")
    amixer_sset(card, "Headphone Mic Boost", "0")

    amixer_sset(card, "Dmic0", "cap")
    amixer_sset(card, "Dmic0", "100%")

    amixer_sset(card, "PGA2.0 2 Master", "50%")
    amixer_sset(card, "PGA4.0 4 Master", "50%")

    amixer_sset(card, "Capture", "cap")
    amixer_sset(card, "Capture", "1", "cap")


def arecord_cmd(device: str, rate: int, channels: int, sec: float, out_wav: Path, fmt: str) -> List[str]:
    dur_s = int(round(sec))
    if dur_s <= 0:
        dur_s = 1
    return [
        "arecord",
        "-D", device,
        "-t", "wav",
        "-c", str(channels),
        "-f", fmt,
        "-r", str(rate),
        "-d", str(dur_s),
        str(out_wav),
    ]


def arecord_record_with_fallback(device: str, rate: int, channels: int, sec: float, out_wav: Path, fmt: str) -> int:
    """
    頑丈な録音:
      - stderr を必ず捕捉
      - 指定chが失敗したら 2ch にフォールバック（DMIC16kHz でよくある）
    """
    out_wav.parent.mkdir(parents=True, exist_ok=True)

    # 1st try
    cmd1 = arecord_cmd(device, rate, channels, sec, out_wav, fmt)
    print("  " + sh(cmd1))
    cp1 = subprocess.run(cmd1, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    if cp1.returncode == 0:
        return channels

    if cp1.stderr:
        print(cp1.stderr.strip(), file=sys.stderr)

    # fallback to 2ch
    if channels != 2:
        print("[WARN] requested channels not supported; fallback to 2ch", file=sys.stderr)
        cmd2 = arecord_cmd(device, rate, 2, sec, out_wav, fmt)
        print("  " + sh(cmd2))
        cp2 = subprocess.run(cmd2, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if cp2.returncode == 0:
            return 2
        if cp2.stderr:
            print(cp2.stderr.strip(), file=sys.stderr)
        raise subprocess.CalledProcessError(cp2.returncode, cmd2, output=cp2.stdout, stderr=cp2.stderr)

    raise subprocess.CalledProcessError(cp1.returncode, cmd1, output=cp1.stdout, stderr=cp1.stderr)


def sox_to_mono(in_wav: Path, out_wav: Path) -> None:
    out_wav.parent.mkdir(parents=True, exist_ok=True)
    cmd = ["sox", str(in_wav), str(out_wav), "channels", "1"]
    print("  " + sh(cmd))
    cp = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if cp.returncode != 0:
        if cp.stderr:
            print(cp.stderr.strip(), file=sys.stderr)
        raise subprocess.CalledProcessError(cp.returncode, cmd, output=cp.stdout, stderr=cp.stderr)


def sox_stat(wav: Path) -> str:
    cp = subprocess.run(
        ["sox", str(wav), "-n", "stat"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if cp.returncode != 0:
        if cp.stderr:
            print(cp.stderr.strip(), file=sys.stderr)
        raise subprocess.CalledProcessError(cp.returncode, ["sox", str(wav), "-n", "stat"], output=cp.stdout, stderr=cp.stderr)
    return cp.stderr


STAT_KEYS = [
    "Samples read",
    "Length (seconds)",
    "Maximum amplitude",
    "Minimum amplitude",
    "Midline amplitude",
    "Mean    norm",
    "Mean    amplitude",
    "RMS     amplitude",
    "Rough   frequency",
    "Volume adjustment",
]


def parse_sox_stat(stat_text: str) -> Dict[str, str]:
    out: Dict[str, str] = {}
    for line in stat_text.splitlines():
        line = line.rstrip()
        for k in STAT_KEYS:
            if line.startswith(k + ":"):
                out[k] = line.split(":", 1)[1].strip()
                break
    return out


def fnum(x: str) -> Optional[float]:
    try:
        return float(x)
    except Exception:
        return None


def judge(stats: Dict[str, str]) -> Tuple[str, str]:
    mx = fnum(stats.get("Maximum amplitude", "nan"))
    rms = fnum(stats.get("RMS     amplitude", "nan"))
    rough = fnum(stats.get("Rough   frequency", "nan"))

    if mx == 0.0 and rms == 0.0:
        return ("SILENCE", "全サンプル0（デバイス指定/ルーティングを確認。DMICは hw:0,6 / hw:0,7）。")

    if mx is not None and mx >= 0.98:
        return ("CLIPPING", "ピークが1.0付近。Boostを0にしてPGAも下げる。")

    if rms is not None and rms >= 0.15:
        note = "RMS大きめ。PGAを下げる。"
        if rough is not None and rough > 800:
            note += "（ノイズ/歪み成分が多い可能性）"
        return ("LOUD", note)

    if rms is not None and rms <= 0.02:
        return ("QUIET", "RMS小さめ。PGAを上げる、必要ならBoostを1まで。")

    note = "概ね適正（目安: Max 0.3〜0.8, RMS 0.03〜0.10）。"
    if rough is not None and rough > 800:
        note += " ただし Rough frequency が高めでノイズっぽいかも。"
    return ("OK", note)


def _clamp_int(x: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, x))


def get_current_pga_percent(card: int, control: str) -> Optional[int]:
    """
    amixer 出力から 'Capture NN [xx%]' の xx を拾う（拾えなければ None）
    """
    txt = amixer_sget(card, control)
    # 例: "Front Left: Capture 50 [62%] [0.00dB]"
    import re
    m = re.search(r"\[(\d+)%\]", txt)
    if not m:
        return None
    try:
        return int(m.group(1))
    except Exception:
        return None


def suggestion_commands(level: str, card: int, delta_pga: int = 5) -> List[str]:
    """
    判定に応じて、次に打つべきコマンドを返す（実行はしない）
    """
    cmds: List[str] = []
    if level == "OK":
        cmds.append("# No change needed.")
        return cmds

    if level == "SILENCE":
        cmds += [
            "# This is not a gain issue. Check device/routing:",
            "arecord -l",
            "arecord -D hw:0,7 -t wav -c 2 -r 16000 -d 3 test.wav",
        ]
        return cmds

    # 現在PGAを読んで、±delta した値を提示（読めなければ固定値）
    pga2 = get_current_pga_percent(card, "PGA2.0 2 Master")
    pga4 = get_current_pga_percent(card, "PGA4.0 4 Master")

    if level == "QUIET":
        # PGA +delta
        target2 = _clamp_int((pga2 if pga2 is not None else 50) + delta_pga, 0, 100)
        target4 = _clamp_int((pga4 if pga4 is not None else 50) + delta_pga, 0, 100)
        cmds += [
            "# Step 1: raise PGA a little",
            f"amixer -c {card} sset 'PGA2.0 2 Master' {target2}%",
            f"amixer -c {card} sset 'PGA4.0 4 Master' {target4}%",
            "",
            "# If still quiet after retry (last resort):",
            f"# amixer -c {card} sset 'Headset Mic Boost' 1",
            f"# amixer -c {card} sset 'Headphone Mic Boost' 1",
        ]
        return cmds

    if level == "LOUD":
        # PGA -delta + Boost off
        target2 = _clamp_int((pga2 if pga2 is not None else 50) - delta_pga, 0, 100)
        target4 = _clamp_int((pga4 if pga4 is not None else 50) - delta_pga, 0, 100)
        cmds += [
            "# Step 1: lower PGA slightly",
            f"amixer -c {card} sset 'PGA2.0 2 Master' {target2}%",
            f"amixer -c {card} sset 'PGA4.0 4 Master' {target4}%",
            "",
            "# Also ensure Boost is off",
            f"amixer -c {card} sset 'Headset Mic Boost' 0",
            f"amixer -c {card} sset 'Headphone Mic Boost' 0",
        ]
        return cmds

    if level == "CLIPPING":
        # Boost off + PGA -2*delta
        target2 = _clamp_int((pga2 if pga2 is not None else 50) - (2 * delta_pga), 0, 100)
        target4 = _clamp_int((pga4 if pga4 is not None else 50) - (2 * delta_pga), 0, 100)
        cmds += [
            "# Turn off Boost immediately",
            f"amixer -c {card} sset 'Headset Mic Boost' 0",
            f"amixer -c {card} sset 'Headphone Mic Boost' 0",
            "",
            "# Then lower PGA",
            f"amixer -c {card} sset 'PGA2.0 2 Master' {target2}%",
            f"amixer -c {card} sset 'PGA4.0 4 Master' {target4}%",
        ]
        return cmds

    # fallback
    cmds.append("# No suggestion available for this level.")
    return cmds


def pretty(stats: Dict[str, str], label: str, card: int, delta_pga: int) -> None:
    print(f"\n[sox stat 抜粋: {label}]")
    for k in [
        "Length (seconds)",
        "Samples read",
        "Maximum amplitude",
        "RMS     amplitude",
        "Mean    norm",
        "Rough   frequency",
        "Volume adjustment",
    ]:
        if k in stats:
            print(f"  {k:<20} : {stats[k]}")

    level, note = judge(stats)
    print("\n[判定]")
    print(f"  Level : {level}")
    print(f"  Note  : {note}")

    print("\n[Suggestion]")
    for line in suggestion_commands(level, card=card, delta_pga=delta_pga):
        print(f"  {line}" if line and not line.startswith("#") else f"{line}")


# ---------------- main ----------------

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--card", type=int, default=0, help="ALSA card index (default: 0)")
    ap.add_argument("--device", default="hw:0,7", help="e.g. hw:0,7(DMIC16kHz) / hw:0,6(DMIC48kHz)")
    ap.add_argument("--rate", type=int, default=16000, help="sample rate (default: 16000)")
    ap.add_argument("--channels", type=int, default=1, help="requested channels (fallback to 2ch on failure)")
    ap.add_argument("--sec", type=float, default=3.0, help="record seconds (rounded to int for arecord -d)")
    ap.add_argument("--out", default="out.wav", help="output wav path")
    ap.add_argument("--format", default="S16_LE", help="arecord format (default: S16_LE)")
    ap.add_argument("--mono", action="store_true", help="after recording, convert to mono and run stat on mono")
    ap.add_argument("--apply-defaults", action="store_true", help="apply safe default mixer settings before record")
    ap.add_argument("--no-amixer", action="store_true", help="do not print amixer controls (faster)")
    ap.add_argument("--delta-pga", type=int, default=5, help="PGA suggestion step size in percent (default: 5)")
    args = ap.parse_args()

    out = Path(args.out).expanduser().resolve()

    if not args.no_amixer:
        show_controls(args.card, "amixer before")

    if args.apply_defaults:
        print("\n[apply defaults]")
        apply_defaults(args.card)
        if not args.no_amixer:
            show_controls(args.card, "amixer after(defaults)")

    print("\n[record]")
    used_ch = arecord_record_with_fallback(args.device, args.rate, args.channels, args.sec, out, args.format)
    print(f"[record] done (used channels = {used_ch}) -> {out}")

    print("\n[sox stat]")
    if args.mono:
        mono = out.with_suffix(".mono.wav")
        print("[mono convert]")
        sox_to_mono(out, mono)
        stats = parse_sox_stat(sox_stat(mono))
        pretty(stats, "mono", card=args.card, delta_pga=args.delta_pga)
    else:
        stats = parse_sox_stat(sox_stat(out))
        pretty(stats, f"{used_ch}ch", card=args.card, delta_pga=args.delta_pga)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

