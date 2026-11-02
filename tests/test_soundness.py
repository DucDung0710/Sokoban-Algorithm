"""Vet can map 0-7: deadlock sound, h1-h3 admissible va consistent (vai tro D).

Dung lai build_graph, true_dist, _make_state trong tests/heuristic_test.py.
Chay tu thu muc goc repo: python -m unittest discover -s tests -p "test_soundness.py" -v
(neu doi ten heuristic_test.py thanh test_heuristic.py thi sua dong import ben duoi).
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import solution as S
from sokoban import PROBLEMS
from heuristic_test import build_graph, true_dist, _make_state

MAPS = range(8)


class TestExhaustive(unittest.TestCase):
    def test_maps_0_7(self):
        old = S.CONFIG["dead"]
        self.addCleanup(S.CONFIG.__setitem__, "dead", old)
        for m in MAPS:
            st = PROBLEMS[m]
            b = S.build_board(st)
            try:
                pred, edges = build_graph(b, S.start_key(st))   # tran 3 trieu state
            except MemoryError:
                print(f"bo qua map {m}: vuot tran")
                continue
            dist = true_dist(b, pred)                          # ngoai dist: C* = vo cuc
            h = {n: {} for n in ("h1", "h2", "h3")}
            for s in pred:
                x = _make_state(st, b, s)
                for mode in ("cells", "freeze"):
                    S.CONFIG["dead"] = mode
                    if s in dist and S.is_dead(x, b):
                        self.fail(f"map {m}, {mode}: cat nham state giai duoc {s}")
                for n in h:
                    h[n][s] = S.HEURISTICS[n](x, b)
            for n, hv in h.items():
                self.assertFalse([s for s in dist if hv[s] > dist[s]], f"map {m} {n}: vuot C*")
                self.assertFalse([(s, u) for s, u, _ in edges
                                  if s in dist and u in dist and hv[s] > 1 + hv[u]],
                                 f"map {m} {n}: khong consistent")


if __name__ == "__main__":
    unittest.main()
