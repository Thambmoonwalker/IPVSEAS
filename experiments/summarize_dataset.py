"""Summarise a whole campaign: one row per condition (scenario, speed, weather, traffic) and
sensor configuration, aggregated over seeds.

    python experiments/summarize_dataset.py datasets/runs/v1_campaign

Writes <campaign>/summary.csv and <campaign>/summary.md, and prints the table. Columns:
  runs        number of recordings in the group
  impacts     runs that ended in a collision (inattentive driver)
  missed      impacts with no WARNING before them
  lead_min / lead_med   warning lead time over the impact runs [s]
  fa_runs     no-impact runs that still had WARNING frames (false alarms)
  recall      mean tracking recall
"""

import argparse
import csv
import json
import statistics
from collections import defaultdict
from pathlib import Path

GROUP_KEYS = ("scenario", "speed", "weather", "traffic")


def fmt(v, nd=2):
    return "-" if v is None else f"{v:.{nd}f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("campaign", type=Path)
    args = ap.parse_args()

    groups = defaultdict(list)
    for cfg_file in sorted(args.campaign.glob("*/run_config.json")):
        run = cfg_file.parent
        cfg = json.loads(cfg_file.read_text())
        key = tuple(str(cfg.get(k, "-")) for k in GROUP_KEYS)
        for ev in run.glob("evaluation*.json"):
            e = json.loads(ev.read_text())
            groups[key + (e.get("tag", "full"),)].append(e)
    if not groups:
        raise SystemExit(f"no evaluated runs under {args.campaign} (run generate_dataset.py --process-only)")

    rows = []
    for key in sorted(groups, key=lambda k: (k[0], float(k[1]) if k[1].replace('.', '').isdigit() else 0, k[2], k[3], k[4])):
        es = groups[key]
        impacts = [e for e in es if e["collision"]]
        leads = [e["lead_time"] for e in impacts if e["lead_time"] is not None]
        recalls = [e["tracking_recall"] for e in es if e.get("tracking_recall") is not None]
        rows.append(dict(zip(GROUP_KEYS + ("sensors",), key),
                         runs=len(es), impacts=len(impacts), missed=sum(e["missed"] for e in impacts),
                         lead_min=min(leads) if leads else None,
                         lead_med=statistics.median(leads) if leads else None,
                         fa_runs=sum(1 for e in es if not e["collision"] and e["false_alarm_frames"] > 0),
                         recall=statistics.mean(recalls) if recalls else None))

    fields = list(GROUP_KEYS) + ["sensors", "runs", "impacts", "missed", "lead_min", "lead_med", "fa_runs", "recall"]
    with open(args.campaign / "summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    header = "| " + " | ".join(fields) + " |\n|" + "---|" * len(fields) + "\n"
    lines = []
    for r in rows:
        vals = [r[k] for k in GROUP_KEYS] + [r["sensors"], r["runs"], r["impacts"], r["missed"],
                                             fmt(r["lead_min"]), fmt(r["lead_med"]), r["fa_runs"],
                                             "-" if r["recall"] is None else f"{r['recall']:.0%}"]
        lines.append("| " + " | ".join(str(v) for v in vals) + " |")
    md = f"# Campaign summary: {args.campaign.name}\n\n" + header + "\n".join(lines) + "\n"
    (args.campaign / "summary.md").write_text(md)
    print(md)
    print(f"wrote {args.campaign / 'summary.csv'} and {args.campaign / 'summary.md'}")


if __name__ == "__main__":
    main()
