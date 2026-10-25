from collections import deque
import time
import unittest

from solution import (
    CONFIG, INF, _minimum_assignment_cost, astar, build_board, h1, h2, h3, h_zero,
    has_frozen_box_deadlock, is_dead, is_dead_static,
)
from sokoban import SokobanState, sokoban_goal_state


def corridor_state(robot, box):
    """Tạo state trong hành lang một ô để tính khoảng cách chính xác."""
    walls = frozenset((x, y) for x in range(5) for y in (0, 2))
    return SokobanState(
        "START", 0, None, 5, 3, (robot,), frozenset((box,)),
        frozenset(((4, 1),)), walls,
    )


def shortest_solution_steps(initial):
    """Tính h* bằng BFS toàn state; O(V·R(R+B)) trong framework này."""
    queue = deque([(initial, 0)])
    visited = {initial.hashable_state()}
    while queue:
        state, distance = queue.popleft()
        if sokoban_goal_state(state):
            return distance
        for child in state.successors():
            key = child.hashable_state()
            if key not in visited:
                visited.add(key)
                queue.append((child, distance + 1))
    return None


class PushDistanceTests(unittest.TestCase):
    def test_assignment_uses_distinct_goals(self):
        """Ghép tối ưu không cho hai hộp dùng chung một đích."""
        self.assertEqual(_minimum_assignment_cost(((1, 5), (1, 100))), 6)
        self.assertEqual(_minimum_assignment_cost(((INF, INF),)), INF)

    def test_h1_h2_h3_are_admissible_for_two_boxes(self):
        """So ba heuristic với BFS chính xác trên các cấu hình hai hộp nhỏ."""
        walls = frozenset((x, y) for x in range(5) for y in (0, 2))
        goals = frozenset(((3, 1), (4, 1)))
        solvable_states = 0

        for robot_x in range(5):
            for first_box_x in range(5):
                for second_box_x in range(first_box_x + 1, 5):
                    if robot_x in (first_box_x, second_box_x):
                        continue
                    state = SokobanState(
                        "START", 0, None, 5, 3, ((robot_x, 1),),
                        frozenset(((first_box_x, 1), (second_box_x, 1))),
                        goals, walls,
                    )
                    board = build_board(state)
                    exact = shortest_solution_steps(state)
                    values = (h1(state, board), h2(state, board), h3(state, board))
                    if exact is None:
                        continue
                    solvable_states += 1
                    for value in values:
                        self.assertLessEqual(value, exact)
                    next_values = []
                    for child in state.successors():
                        child_board = board
                        child_values = (
                            h1(child, child_board),
                            h2(child, child_board),
                            h3(child, child_board),
                        )
                        next_values.append(child_values)
                    for value_index, value in enumerate(values):
                        for child_values in next_values:
                            self.assertLessEqual(
                                value, 1 + child_values[value_index])

        self.assertGreater(solvable_states, 0)

    def test_h2_cache_obeys_configured_limit(self):
        old_limit = CONFIG["h_cache_max"]
        try:
            CONFIG["h_cache_max"] = 1
            first = corridor_state((0, 1), (1, 1))
            second = corridor_state((1, 1), (2, 1))
            board = build_board(first)
            first_value = h2(first, board)
            self.assertEqual(h2(first, board), first_value)
            self.assertEqual(len(board.h_cache), 1)
            h2(second, board)
            self.assertLessEqual(len(board.h_cache), 1)
        finally:
            CONFIG["h_cache_max"] = old_limit

    def test_reverse_push_requires_robot_support_square(self):
        """Ô sau thùng phải sàn; góc không thể đẩy ra được đánh dấu dead."""
        state = corridor_state((0, 1), (1, 1))
        board = build_board(state)

        self.assertEqual(board.pdc[0][4 + board.W], 0)
        self.assertEqual(board.pd_min[1 + board.W], 3)
        self.assertEqual(board.pd_min[0 + board.W], INF)
        self.assertIn(0 + board.W, board.dead)

    def test_h1_is_admissible_and_consistent_on_small_board(self):
        """So h1 với h* và kiểm mọi cạnh trong không gian một hộp nhỏ."""
        floor_cells = tuple((x, 1) for x in range(5))
        checked_solvable = 0
        checked_edges = 0

        for robot in floor_cells:
            for box in floor_cells:
                if robot == box:
                    continue
                state = corridor_state(robot, box)
                board = build_board(state)
                value = h1(state, board)
                assignment_value = h2(state, board)
                combined_value = h3(state, board)
                exact = shortest_solution_steps(state)

                if value == INF:
                    self.assertIsNone(exact, "Dead square must be unsolvable")
                elif exact is not None:
                    self.assertLessEqual(value, exact, "h1 must not exceed h*")
                    self.assertGreaterEqual(assignment_value, value)
                    self.assertLessEqual(assignment_value, exact)
                    self.assertLessEqual(combined_value, exact)
                    checked_solvable += 1

                for child in state.successors():
                    checked_edges += 1
                    child_value = h1(child, board)
                    if value == INF:
                        self.assertEqual(child_value, INF)
                    else:
                        self.assertLessEqual(value, 1 + child_value)

        self.assertGreater(checked_solvable, 0)
        self.assertGreater(checked_edges, 0)

    def test_solve_uses_push_distance_heuristic(self):
        """Kiểm tra A* tìm được lời giải hợp lệ với h1."""
        from solution import solve

        initial = corridor_state((0, 1), (1, 1))
        goal = solve(initial, timebound=5)
        self.assertIsInstance(goal, SokobanState)
        self.assertTrue(sokoban_goal_state(goal))
        self.assertEqual(goal.gval, 3)


class StaticDeadlockTests(unittest.TestCase):
    def test_static_dead_square_is_detected_and_pruned(self):
        """Ô góc không thể tới đích được nhận diện và A* loại bỏ."""
        initial = corridor_state((2, 1), (1, 1))
        board = build_board(initial)
        dead_state = corridor_state((1, 1), (0, 1))

        self.assertTrue(is_dead_static(dead_state, board))
        self.assertFalse(is_dead_static(initial, board))

        goal, stats = astar(
            initial,
            time.monotonic() + 5,
            h_zero,
            lambda state: is_dead_static(state, board),
        )
        self.assertIsNone(goal)
        self.assertGreater(stats["pruned_dead"], 0)

    def test_box_frozen_by_walls_is_detected(self):
        """Hộp bị tường khóa cả hai trục bị phát hiện, trừ khi nằm trên đích."""
        walls = frozenset(((1, 2), (3, 2), (2, 1), (2, 3)))
        initial = SokobanState(
            "START", 0, None, 5, 5, ((3, 2),),
            frozenset(((2, 2),)), frozenset(((4, 4),)), walls,
        )
        board = build_board(initial)

        self.assertTrue(has_frozen_box_deadlock(initial, board))
        self.assertTrue(is_dead(initial, board))
        goal, stats = astar(
            initial, time.monotonic() + 5, h_zero,
            lambda state: is_dead(state, board),
        )
        self.assertIsNone(goal)
        self.assertEqual(stats["reason"], "deadlock_at_start")

        box_on_goal = SokobanState(
            "START", 0, None, 5, 5, ((3, 2),),
            frozenset(((2, 2),)), frozenset(((2, 2),)), walls,
        )
        self.assertFalse(
            has_frozen_box_deadlock(box_on_goal, build_board(box_on_goal)))

    def test_deadlock_modes_can_be_disabled_or_selected(self):
        walls = frozenset(((1, 2), (3, 2), (2, 1), (2, 3)))
        state = SokobanState(
            "START", 0, None, 5, 5, ((0, 0),),
            frozenset(((2, 2),)), frozenset(((4, 4),)), walls,
        )
        board = build_board(state)
        old_mode = CONFIG["dead"]
        try:
            CONFIG["dead"] = "none"
            self.assertFalse(is_dead(state, board))
            CONFIG["dead"] = "cells"
            self.assertTrue(is_dead(state, board))
            CONFIG["dead"] = "freeze"
            self.assertTrue(is_dead(state, board))
        finally:
            CONFIG["dead"] = old_mode

    def test_pushed_box_freeze_check_is_local(self):
        walls = frozenset(((1, 2), (3, 2), (2, 1), (2, 3)))
        state = SokobanState(
            "START", 0, None, 7, 5, ((0, 0),),
            frozenset(((2, 2), (5, 2))),
            frozenset(((4, 4),)), walls,
        )
        board = build_board(state)
        old_mode = CONFIG["dead"]
        try:
            CONFIG["dead"] = "freeze"
            self.assertFalse(is_dead(state, board, pushed=5 + 2 * board.W))
            self.assertTrue(is_dead(state, board, pushed=2 + 2 * board.W))
        finally:
            CONFIG["dead"] = old_mode

    def test_deadlock_detector_is_sound_on_small_corridor(self):
        old_mode = CONFIG["dead"]
        try:
            CONFIG["dead"] = "freeze"
            for robot_x in range(5):
                for box_x in range(5):
                    if robot_x == box_x:
                        continue
                    state = corridor_state((robot_x, 1), (box_x, 1))
                    if is_dead(state, build_board(state)):
                        self.assertIsNone(shortest_solution_steps(state))
        finally:
            CONFIG["dead"] = old_mode


if __name__ == "__main__":
    unittest.main()
