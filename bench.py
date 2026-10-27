"""Benchmark solution.py tren 20 map PROBLEMS, moi map mot tien trinh rieng.

Chay duoc tren Linux, macOS va Windows (khong can thu vien ngoai).

Cach dung:
    python bench.py <thu_muc_solution> <timebound> <file_ket_qua.json> <so_tien_trinh> [ghi_de] [cac_map]

    ghi_de  : JSON ghi de bien module, vi du {"CONFIG": {"h": "h3"}, "WEIGHTS": [1.0], "MAX_STATES": 6000000}
              hoac @duong_dan.json de doc tu file (tien tren Windows, khoi lo chuyen dau nhay).
    cac_map : danh sach so map cach nhau dau phay, vi du 10,15,16 (mac dinh ca 20 map)

Vi du:
    python bench.py . 120 runs/final_120.json 2
    python bench.py . 60 runs/abl_h3_60.json 2 @runs/abl_h3.json
"""
import json
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent

CHILD = r'''
import json, sys, time

def peak_rss_mb():
    """RAM dinh cua tien trinh hien tai (MB), hoac None neu khong do duoc."""
    try:
        if sys.platform == "win32":
            import ctypes
            from ctypes import wintypes

            class PMC(ctypes.Structure):
                _fields_ = [("cb", wintypes.DWORD), ("PageFaultCount", wintypes.DWORD),
                            ("PeakWorkingSetSize", ctypes.c_size_t), ("WorkingSetSize", ctypes.c_size_t),
                            ("QuotaPeakPagedPoolUsage", ctypes.c_size_t), ("QuotaPagedPoolUsage", ctypes.c_size_t),
                            ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t), ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
                            ("PagefileUsage", ctypes.c_size_t), ("PeakPagefileUsage", ctypes.c_size_t)]

            k32 = ctypes.WinDLL("kernel32")
            k32.GetCurrentProcess.restype = wintypes.HANDLE
            try:
                fn = ctypes.WinDLL("psapi").GetProcessMemoryInfo
            except (OSError, AttributeError):
                fn = k32.K32GetProcessMemoryInfo
            fn.argtypes = [wintypes.HANDLE, ctypes.POINTER(PMC), wintypes.DWORD]
            fn.restype = wintypes.BOOL
            pmc = PMC()
            pmc.cb = ctypes.sizeof(PMC)
            if not fn(k32.GetCurrentProcess(), ctypes.byref(pmc), pmc.cb):
                return None
            return pmc.PeakWorkingSetSize // (1024 * 1024)
        import resource
        peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        return peak // (1024 * 1024) if sys.platform == "darwin" else peak // 1024
    except Exception:
        return None

sys.path.insert(0, sys.argv[1]); sys.path.insert(1, sys.argv[2])
import solution as S
ov = json.loads(sys.argv[5])
if "CONFIG" in ov: S.CONFIG.update(ov["CONFIG"])
if "WEIGHTS" in ov: S.WEIGHTS = tuple(ov["WEIGHTS"])
if "MAX_STATES" in ov: S.MAX_STATES = ov["MAX_STATES"]
from sokoban import PROBLEMS
from validation import validate_solution
i, tb = int(sys.argv[3]), float(sys.argv[4])
st = PROBLEMS[i]
t0 = time.monotonic()
res = S.solve(st, timebound=tb)
dt = time.monotonic() - t0
ok, msg = (validate_solution(st, res) if res is not False else (False, "returned False"))
info = getattr(S, "LAST_INFO", {})
print(json.dumps(dict(map=i, solved=bool(ok), steps=(res.gval if ok else None),
                      time=round(dt, 2), rss_mb=peak_rss_mb(), msg=msg,
                      proved=info.get("proved"), phases=info.get("phases"))))
'''


def run(sol_dir, i, tb, ov):
    try:
        p = subprocess.run([sys.executable, "-c", CHILD, str(sol_dir), str(ROOT), str(i), str(tb), ov],
                           capture_output=True, text=True, errors="replace", timeout=tb + 60)
        line = p.stdout.strip().splitlines()[-1] if p.stdout.strip() else ""
        if p.returncode != 0 or not line:
            return dict(map=i, solved=False, steps=None, time=None, rss_mb=None,
                        msg="CRASH rc=%s %s" % (p.returncode, p.stderr.strip()[-300:]))
        return json.loads(line)
    except subprocess.TimeoutExpired:
        return dict(map=i, solved=False, steps=None, time=None, rss_mb=None, msg="HARD TIMEOUT")


def load_overrides(arg):
    """Doc ghi de tu chuoi JSON hoac tu file khi bat dau bang @."""
    text = Path(arg[1:]).read_text(encoding="utf-8") if arg.startswith("@") else arg
    return json.dumps(json.loads(text))  # kiem tra JSON hop le truoc khi chay


def main():
    if len(sys.argv) < 5:
        print(__doc__)
        return 2
    sol_dir, tb, out = Path(sys.argv[1]).resolve(), float(sys.argv[2]), Path(sys.argv[3])
    workers = int(sys.argv[4])
    ov = load_overrides(sys.argv[5]) if len(sys.argv) > 5 else "{}"
    maps = [int(x) for x in sys.argv[6].split(",")] if len(sys.argv) > 6 else list(range(20))
    out.parent.mkdir(parents=True, exist_ok=True)
    with ThreadPoolExecutor(workers) as ex:
        results = list(ex.map(lambda i: run(sol_dir, i, tb, ov), maps))
    out.write_text(json.dumps(results, indent=1), encoding="utf-8")
    for r in results:
        print("map %2d  %-4s steps=%-4s t=%-6s rss=%-5sMB proved=%s  %s" % (
            r["map"], "OK" if r["solved"] else "FAIL", r["steps"], r["time"], r["rss_mb"],
            r.get("proved"), "" if r["solved"] else r["msg"]))
    print("solved %d/%d" % (sum(r["solved"] for r in results), len(results)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
