#!/usr/bin/env python3
"""In bảng thống kê từ file kết quả của bench.py (runs/*.json).

Chỉ đọc JSON mà bench.py đã ghi: không chạy solver, không sửa solution.py.
Đặt cạnh bench.py ở thư mục gốc repo. Không nộp file này (bài nộp chỉ có solution.py).

Cách dùng (Windows: thay python3 bằng py):
    python3 make_table.py runs/final_120.json
    python3 make_table.py runs/final_120.json --md runs/table_final.md --csv runs/table_final.csv
    python3 make_table.py runs/demo_baseline_120.json runs/final_120.json --label Demo "Đề xuất"
    python3 make_table.py runs/final_120.json "runs/abl_*.json" --maps 9-19 --md runs/ablation.md

Một file   -> bảng chi tiết từng map + dòng tổng.
Nhiều file -> bảng số bước và bảng thời gian theo map, cộng bảng tổng kết mỗi lần chạy.
              File đầu tiên là mốc so sánh cho các cột Δ.
--maps   : chỉ lấy một số map, ví dụ 9-19 hoặc 0,3,15-19.
--label  : tên hiển thị cho từng file (mặc định là tên file).
--md     : ghi Markdown, số kiểu Việt Nam (1.234 và 66,4), dán thẳng vào mục 3 REPORT_TEMPLATE.md.
--csv    : ghi CSV dạng dài (mỗi dòng là một map của một lần chạy), số kiểu máy (66.4), mở bằng Excel.
--note   : một dòng ghi cấu hình máy đo, in dưới bảng Markdown.
"""
import argparse
import csv
import glob
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DASH = "—"

STATUS_VI = {
    "PASS": "Đạt",
    "NO_SOLUTION": "Không ra",
    "INVALID": "Sai đường",
    "TIMEOUT": "Quá giờ",
    "CRASH": "Lỗi",
    "MUTATED": "Sửa input",
}


# ---------------------------------------------------------------- đọc dữ liệu

def status_of(r):
    """Trạng thái chuẩn hoá từ một dòng của bench.py (bản 27/10 hoặc bản cũ có trường status)."""
    if r.get("status"):
        return str(r["status"]).upper()
    if r.get("solved"):
        return "PASS"
    msg = str(r.get("msg") or "")
    if msg.startswith("CRASH"):
        return "CRASH"
    if "TIMEOUT" in msg.upper():
        return "TIMEOUT"
    if msg == "returned False":
        return "NO_SOLUTION"
    return "INVALID"


def normalize(r):
    phases = [p for p in (r.get("phases") or []) if isinstance(p, dict)]
    status = status_of(r)
    steps = r.get("steps") if status == "PASS" else None
    expanded = sum(p.get("expanded") or 0 for p in phases) if phases else None
    sizes = [p.get("states", p.get("generated")) for p in phases]
    states = max((s for s in sizes if s is not None), default=None)
    weight = None
    for p in phases:  # pha tìm ra lời giải cuối cùng là pha có cost bằng số bước
        if p.get("cost") is not None and p.get("cost") == steps:
            weight = p.get("weight")
    return dict(map=int(r["map"]), status=status, steps=steps,
                proved=r.get("proved") if status == "PASS" else None,
                time=r.get("time"), rss=r.get("rss_mb"), expanded=expanded,
                states=states, weight=weight,
                msg=str(r.get("msg") or r.get("detail") or ""))


def load_run(path):
    data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if isinstance(data, dict):
        data = data.get("results", [])
    recs = {}
    for r in data:
        if isinstance(r, dict) and "map" in r:
            n = normalize(r)
            recs[n["map"]] = n
    return recs


def load_map_meta():
    """Kích thước, số robot/thùng/ô đích của 20 map, lấy từ sokoban.py cạnh file này."""
    sys.path.insert(0, str(ROOT))
    try:
        from sokoban import PROBLEMS
    except Exception:
        return {}
    return {i: dict(w=s.width, h=s.height, robots=len(s.robots), boxes=len(s.boxes),
                    storage=len(s.storage)) for i, s in enumerate(PROBLEMS)}


# ---------------------------------------------------------------- định dạng số

def f_int(v):
    return DASH if v is None else "{:,}".format(int(v)).replace(",", ".")


def f_time(v):
    if v is None:
        return DASH
    return ("{:.2f}" if v < 10 else "{:.1f}").format(v).replace(".", ",")


def f_w(v):
    return DASH if v is None else "{:g}".format(v).replace(".", ",")


def f_proved(v):
    return {True: "Có", False: "Chưa"}.get(v, DASH)


def f_status(r):
    return STATUS_VI.get(r["status"], r["status"]) if r else DASH


def span(nums):
    """[0,1,2,3,9,11,12] -> '0–3, 9, 11–12'."""
    out, nums = [], sorted(nums)
    i = 0
    while i < len(nums):
        j = i
        while j + 1 < len(nums) and nums[j + 1] == nums[j] + 1:
            j += 1
        out.append(str(nums[i]) if i == j else "{}–{}".format(nums[i], nums[j]))
        i = j + 1
    return ", ".join(out)


def md_escape(s):
    return str(s).replace("|", "\\|")


def render(headers, rows, align, md, total_rows=0):
    """Bảng Markdown hoặc bảng căn cột cho terminal. align: 'l' hoặc 'r' cho từng cột."""
    if md:
        lines = ["| " + " | ".join(md_escape(h) for h in headers) + " |",
                 "| " + " | ".join("---:" if a == "r" else "---" for a in align) + " |"]
        lines += ["| " + " | ".join(md_escape(c) for c in row) + " |" for row in rows]
        return "\n".join(lines)
    widths = [max([len(h)] + [len(row[i]) for row in rows]) for i, h in enumerate(headers)]

    def line(cells):
        return "  ".join(c.rjust(w) if a == "r" else c.ljust(w)
                         for c, w, a in zip(cells, widths, align)).rstrip()

    rule = "  ".join("-" * w for w in widths)
    body = [line(row) for row in rows]
    if total_rows:
        body.insert(len(body) - total_rows, rule)
    return "\n".join([line(headers), rule] + body)


def drop_empty_columns(headers, rows, align, keep, n_data):
    """Bỏ cột mà mọi dòng dữ liệu đều trống (ví dụ solver không ghi LAST_INFO)."""
    cols = [i for i, h in enumerate(headers)
            if h in keep or any(row[i] != DASH for row in rows[:n_data])]
    pick = lambda seq: [seq[i] for i in cols]
    return pick(headers), [pick(row) for row in rows], pick(align)


def bold_best(values, md, as_text):
    """Chuỗi hiển thị cho một hàng; in đậm giá trị nhỏ nhất khi các lần chạy khác nhau."""
    nums = [v for v in values if isinstance(v, (int, float))]
    best = min(nums) if md and len(set(nums)) > 1 else None
    out = []
    for v in values:
        s = as_text(v) if isinstance(v, (int, float)) else v
        out.append("**{}**".format(s) if best is not None and v == best else s)
    return out


# ---------------------------------------------------------------- một lần chạy

def single_report(run, meta, md, note):
    recs = run["recs"]
    maps = sorted(recs)
    has_meta = bool(meta) and all(m in meta for m in maps)
    headers = ["Map"]
    align = ["r"]
    if has_meta:
        headers += ["Kích thước", "Robot", "Thùng", "Ô đích"]
        align += ["l", "r", "r", "r"]
    headers += ["Kết quả", "Số bước", "Tối ưu", "Thời gian (s)", "RAM (MB)",
                "Node mở rộng", "State lưu", "w"]
    align += ["l", "r", "l", "r", "r", "r", "r", "r"]

    rows = []
    for m in maps:
        r = recs[m]
        row = [str(m)]
        if has_meta:
            g = meta[m]
            row += ["{}×{}".format(g["w"], g["h"]), str(g["robots"]), str(g["boxes"]), str(g["storage"])]
        row += [f_status(r), f_int(r["steps"]), f_proved(r["proved"]), f_time(r["time"]),
                f_int(r["rss"]), f_int(r["expanded"]), f_int(r["states"]), f_w(r["weight"])]
        rows.append(row)

    n = len(maps)
    ok = [recs[m] for m in maps if recs[m]["status"] == "PASS"]
    times = [recs[m]["time"] for m in maps if recs[m]["time"] is not None]
    rss = [recs[m]["rss"] for m in maps if recs[m]["rss"] is not None]
    exps = [recs[m]["expanded"] for m in maps if recs[m]["expanded"] is not None]
    sts = [recs[m]["states"] for m in maps if recs[m]["states"] is not None]
    proved_known = any(recs[m]["proved"] is not None for m in maps)
    total = ["Tổng"] + ([""] * 4 if has_meta else [])
    total += ["{}/{} đạt".format(len(ok), n), f_int(sum(r["steps"] for r in ok) if ok else None),
              "{}/{}".format(sum(1 for r in ok if r["proved"]), n) if proved_known else DASH,
              f_time(sum(times) if times else None), f_int(max(rss) if rss else None),
              f_int(sum(exps) if exps else None), f_int(max(sts) if sts else None), ""]
    if md:
        total = ["**{}**".format(c) if c else c for c in total]
    rows.append(total)

    keep = {"Map", "Kết quả", "Số bước", "Thời gian (s)"}
    headers, rows, align = drop_empty_columns(headers, rows, align, keep, n)
    table = render(headers, rows, align, md, total_rows=1)

    notes = []
    timed = [recs[m] for m in maps if recs[m]["time"] is not None]
    if timed:
        slow = max(timed, key=lambda r: r["time"])
        notes.append("Chậm nhất: map {} ({} s).".format(slow["map"], f_time(slow["time"])))
    if rss:
        top = max((recs[m] for m in maps if recs[m]["rss"] is not None), key=lambda r: r["rss"])
        notes.append("RAM đỉnh: map {} ({} MB).".format(top["map"], f_int(top["rss"])))
    for status in STATUS_VI:  # gom map không đạt theo trạng thái; lỗi lạ thì kèm thông báo
        bad = [recs[m] for m in maps if recs[m]["status"] == status and status != "PASS"]
        if not bad:
            continue
        if status in ("NO_SOLUTION", "TIMEOUT"):
            items = span(r["map"] for r in bad)
        else:
            items = "; ".join("{} ({})".format(r["map"], r["msg"][-100:].replace("\n", " ") or DASH)
                              for r in bad)
        notes.append("{}: map {}.".format(STATUS_VI[status], items))
    others = [recs[m] for m in maps if recs[m]["status"] not in STATUS_VI]
    if others:
        notes.append("Trạng thái khác: " + ", ".join(
            "map {} ({})".format(r["map"], r["status"]) for r in others) + ".")

    if not md:
        return "\n".join(["Kết quả: {} ({})".format(run["label"], run["file"]), "", table, ""] + notes)
    out = ["### Kết quả trên {} map — {}".format(n, md_escape(run["label"])), "", table, ""]
    out.append(" ".join(notes))
    out += ["", "*Tối ưu*: Có = solver đã chứng minh lời giải là tối ưu; Chưa = có lời giải nhưng "
            "chưa chứng minh. *w*: trọng số của pha tìm ra lời giải cuối. *Node mở rộng* cộng mọi "
            "pha; *State lưu* lấy pha lớn nhất. Dòng Tổng: số bước cộng các map đạt, thời gian cộng "
            "mọi map, RAM và State lấy giá trị lớn nhất."]
    out.append("")
    out.append("Nguồn: `{}`.".format(run["file"]) + (" Cấu hình đo: {}.".format(note.rstrip(".")) if note else ""))
    return "\n".join(out)


# ---------------------------------------------------------------- nhiều lần chạy

def compare_report(runs, meta, md, note):
    maps = sorted(set().union(*(r["recs"] for r in runs)))
    labels = [r["label"] for r in runs]
    two = len(runs) == 2

    def cell(run, m, key):
        r = run["recs"].get(m)
        if r is None:
            return DASH
        if r["status"] != "PASS":  # map không đạt: ghi trạng thái, không so số
            return f_status(r)
        v = r[key]
        return DASH if v is None else v

    # bảng số bước
    h1 = ["Map"] + labels + (["Δ"] if two else [])
    a1 = ["r"] * len(h1)
    rows1 = []
    for m in maps:
        vals = [cell(run, m, "steps") for run in runs]
        row = [str(m)] + bold_best(vals, md, f_int)
        if two:
            d = (vals[1] - vals[0]) if all(isinstance(v, int) for v in vals) else None
            row.append(DASH if d is None else "{:+d}".format(d) if d else "0")
        rows1.append(row)

    # bảng thời gian
    h2 = ["Map"] + labels
    a2 = ["r"] * len(h2)
    rows2 = [[str(m)] + bold_best([cell(run, m, "time") for run in runs], md, f_time) for m in maps]

    # bảng tổng kết
    # bảng tổng kết: cột có dấu * chỉ cộng trên các map mà mọi lần chạy đều đạt
    common = [m for m in maps if all(run["recs"].get(m, {}).get("status") == "PASS" for run in runs)]
    h3 = ["Lần chạy", "Đạt", "Tối ưu (CM)", "Bước*", "Δ bước*", "Thời gian (s)*",
          "Node mở rộng*", "RAM đỉnh (MB)"]
    a3 = ["l", "r", "r", "r", "r", "r", "r", "r"]
    rows3, base = [], None

    def total(run, key):
        vals = [run["recs"][m][key] for m in common]
        return sum(vals) if vals and None not in vals else None

    for k, run in enumerate(runs):
        recs = [run["recs"][m] for m in maps if m in run["recs"]]
        ok = [r for r in recs if r["status"] == "PASS"]
        steps = total(run, "steps")
        if k == 0:
            base = steps
        rss = [r["rss"] for r in recs if r["rss"] is not None]
        known = any(r["proved"] is not None for r in recs)
        if k == 0 or steps is None or base is None:
            delta = DASH
        else:
            delta = "{:+d}".format(steps - base) if steps != base else "0"
        rows3.append([run["label"], "{}/{}".format(len(ok), len(recs)),
                      "{}/{}".format(sum(1 for r in ok if r["proved"]), len(recs)) if known else DASH,
                      f_int(steps), delta, f_time(total(run, "time")), f_int(total(run, "expanded")),
                      f_int(max(rss) if rss else None)])
    h3, rows3, a3 = drop_empty_columns(h3, rows3, a3, {"Lần chạy", "Đạt", "Bước*"}, len(rows3))
    star = ("* cộng trên {} map mà mọi lần chạy đều đạt ({}), để so sánh công bằng."
            .format(len(common), span(common)) if common else "* không có map nào mà mọi lần chạy đều đạt.")

    t1, t2, t3 = (render(h1, rows1, a1, md), render(h2, rows2, a2, md), render(h3, rows3, a3, md))
    if not md:
        src = ["  {} = {}".format(r["label"], r["file"]) for r in runs]
        return "\n".join(["So sánh {} lần chạy (mốc: {})".format(len(runs), labels[0])] + src +
                         ["", "Số bước theo map", t1, "", "Thời gian theo map (s)", t2, "",
                          "Tổng kết", t3, star])
    out = ["### So sánh {} lần chạy".format(len(runs)), "",
           "#### Tổng kết", "", t3, "",
           star.replace("* ", "\\* ", 1) + " Δ tính so với `{}`. *Tối ưu (CM)*: số map solver chứng minh "
           "được lời giải tối ưu. RAM đỉnh lấy trên mọi map.".format(md_escape(labels[0])),
           "", "#### Số bước theo map", "", t1, "",
           "#### Thời gian theo map (s)", "", t2, "",
           "In đậm: tốt nhất trong hàng. Ô ghi chữ (Quá giờ, Không ra…) là map không đạt.", "",
           "Nguồn: " + ", ".join("`{}` = `{}`".format(md_escape(r["label"]), r["file"]) for r in runs) + "."
           + (" Cấu hình đo: {}.".format(note.rstrip(".")) if note else "")]
    return "\n".join(out)


# ---------------------------------------------------------------- CSV và main

CSV_COLUMNS = ["run", "file", "map", "width", "height", "robots", "boxes", "storage", "status",
               "steps", "proved", "time_s", "rss_mb", "expanded", "states", "weight", "msg"]


def write_csv(path, runs, meta):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8-sig") as fh:  # BOM để Excel đọc đúng tiếng Việt
        wr = csv.writer(fh)
        wr.writerow(CSV_COLUMNS)
        for run in runs:
            for m in sorted(run["recs"]):
                r, g = run["recs"][m], meta.get(m, {})
                wr.writerow([run["label"], run["file"], m, g.get("w", ""), g.get("h", ""),
                             g.get("robots", ""), g.get("boxes", ""), g.get("storage", ""),
                             r["status"], r["steps"], r["proved"], r["time"], r["rss"],
                             r["expanded"], r["states"], r["weight"], r["msg"]])


def parse_maps(text):
    out = set()
    try:
        for part in text.split(","):
            part = part.strip()
            if "-" in part:
                lo, hi = part.split("-", 1)
                out.update(range(int(lo), int(hi) + 1))
            elif part:
                out.add(int(part))
    except ValueError:
        raise argparse.ArgumentTypeError("--maps phải có dạng 9-19 hoặc 0,3,15-19")
    return out


def expand_files(patterns, parser):
    """Tự mở rộng dấu * vì PowerShell/cmd không làm việc này."""
    files = []
    for p in patterns:
        if any(ch in p for ch in "*?[") and not Path(p).exists():
            hits = sorted(glob.glob(p))
            if not hits:
                parser.error("Không có file nào khớp: " + p)
            files += hits
        elif Path(p).is_file():
            files.append(p)
        else:
            parser.error("Không tìm thấy file: " + p)
    seen = set()
    return [f for f in files if not (f in seen or seen.add(f))]


def main(argv=None):
    for stream in (sys.stdout, sys.stderr):  # để in được tiếng Việt trên Windows/Git Bash
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+", help="file JSON do bench.py ghi, ví dụ runs/final_120.json")
    ap.add_argument("--label", nargs="+", help="tên hiển thị cho từng file, theo đúng thứ tự")
    ap.add_argument("--maps", type=parse_maps, help="chỉ lấy các map này, ví dụ 9-19")
    ap.add_argument("--md", type=Path, help="ghi bảng Markdown ra file này")
    ap.add_argument("--csv", type=Path, help="ghi CSV dạng dài ra file này")
    ap.add_argument("--note", help='cấu hình máy đo, ví dụ "Ubuntu 24.04, i5-1135G7, 16 GB RAM, Python 3.12"')
    args = ap.parse_args(argv)

    files = expand_files(args.files, ap)
    labels = args.label or [Path(f).stem for f in files]
    if len(labels) != len(files):
        ap.error("Có {} file nhưng {} nhãn --label".format(len(files), len(labels)))

    runs = []
    for f, lab in zip(files, labels):
        try:
            recs = load_run(f)
        except (OSError, ValueError) as exc:
            ap.error("Không đọc được {}: {}".format(f, exc))
        if args.maps is not None:
            recs = {m: r for m, r in recs.items() if m in args.maps}
        if not recs:
            ap.error("{}: không có kết quả map nào{}".format(f, " trong --maps" if args.maps else ""))
        runs.append(dict(label=lab, file=Path(f).as_posix(), recs=recs))

    meta = load_map_meta()
    build = single_report if len(runs) == 1 else compare_report
    target = runs[0] if len(runs) == 1 else runs
    print(build(target, meta, md=False, note=args.note))

    if args.md:
        args.md.parent.mkdir(parents=True, exist_ok=True)
        args.md.write_text(build(target, meta, md=True, note=args.note) + "\n", encoding="utf-8")
        print("\nĐã ghi Markdown: " + str(args.md))
    if args.csv:
        write_csv(args.csv, runs, meta)
        print(("" if args.md else "\n") + "Đã ghi CSV: " + str(args.csv))
    return 0


if __name__ == "__main__":
    sys.exit(main())
