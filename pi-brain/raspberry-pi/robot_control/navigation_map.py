"""可标定的场地路网、动态障碍封路和 A* 全局改道。"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import json
import math
from pathlib import Path
from typing import Dict, Iterable, Mapping, Optional, Sequence


@dataclass(frozen=True)
class Point2D:
    x_mm: float
    y_mm: float


@dataclass(frozen=True)
class Pose2D(Point2D):
    yaw_rad: float = 0.0


@dataclass(frozen=True)
class MapNode(Point2D):
    name: str = ""


@dataclass(frozen=True)
class RoadEdge:
    name: str
    start: str
    end: str
    width_mm: float


@dataclass(frozen=True)
class CircularObstacle(Point2D):
    radius_mm: float
    observed_at: float = 0.0


@dataclass(frozen=True)
class RoutePlan:
    nodes: tuple[str, ...]
    blocked_edges: frozenset[str]
    length_mm: float


class NoRouteError(RuntimeError):
    pass


class NavigationMap:
    """田字形拓扑地图。

    图中的坐标是赛前初值；在正式场地测量后只需修改 JSON，不修改算法。
    动态障碍只影响当前被判定为不可安全通过的边，不修改静态地图。
    """

    def __init__(
        self,
        *,
        field_width_mm: float,
        field_height_mm: float,
        robot_radius_mm: float,
        safety_margin_mm: float,
        nodes: Mapping[str, MapNode],
        edges: Sequence[RoadEdge],
        targets: Mapping[str, str],
        initial_node: str,
        turn_penalty_mm: float = 80.0,
    ) -> None:
        if field_width_mm <= 0 or field_height_mm <= 0:
            raise ValueError("场地尺寸必须大于 0")
        if robot_radius_mm <= 0 or safety_margin_mm < 0:
            raise ValueError("机器人半径必须大于 0，安全余量不能为负")
        self.field_width_mm = field_width_mm
        self.field_height_mm = field_height_mm
        self.robot_radius_mm = robot_radius_mm
        self.safety_margin_mm = safety_margin_mm
        self.nodes = dict(nodes)
        self.edges = tuple(edges)
        self.targets = dict(targets)
        self.initial_node = initial_node
        self.turn_penalty_mm = turn_penalty_mm
        if initial_node not in self.nodes:
            raise ValueError(f"initial_node 不存在：{initial_node}")
        for edge in self.edges:
            if edge.start not in self.nodes or edge.end not in self.nodes:
                raise ValueError(f"道路 {edge.name} 引用了不存在的节点")
            if edge.width_mm <= 0:
                raise ValueError(f"道路 {edge.name} 宽度必须大于 0")
        for target, node in self.targets.items():
            if node not in self.nodes:
                raise ValueError(f"目标 {target} 引用了不存在的节点 {node}")

    @classmethod
    def load(cls, path: str | Path) -> "NavigationMap":
        source = Path(path)
        data = json.loads(source.read_text(encoding="utf-8"))
        field = data["field"]
        robot = data["robot"]
        nodes = {
            name: MapNode(float(value[0]), float(value[1]), name)
            for name, value in data["nodes"].items()
        }
        edges = [
            RoadEdge(
                str(item["name"]),
                str(item["start"]),
                str(item["end"]),
                float(item["width_mm"]),
            )
            for item in data["edges"]
        ]
        return cls(
            field_width_mm=float(field["width_mm"]),
            field_height_mm=float(field["height_mm"]),
            robot_radius_mm=float(robot["footprint_radius_mm"]),
            safety_margin_mm=float(robot["safety_margin_mm"]),
            nodes=nodes,
            edges=edges,
            targets=data["targets"],
            initial_node=str(data["start"]["initial_node"]),
            turn_penalty_mm=float(data["planner"].get("turn_penalty_mm", 80.0)),
        )

    @property
    def clearance_mm(self) -> float:
        return self.robot_radius_mm + self.safety_margin_mm

    def target_node(self, target: object) -> str:
        key = getattr(target, "value", target)
        try:
            return self.targets[str(key)]
        except KeyError as error:
            raise ValueError(f"地图未配置导航目标：{key}") from error

    def nearest_node(self, point: Point2D) -> str:
        return min(
            self.nodes,
            key=lambda name: _distance(point, self.nodes[name]),
        )

    def contains_footprint(self, pose: Point2D) -> bool:
        clearance = self.clearance_mm
        return (
            clearance <= pose.x_mm <= self.field_width_mm - clearance
            and clearance <= pose.y_mm <= self.field_height_mm - clearance
        )

    def blocked_edges(self, obstacles: Iterable[CircularObstacle]) -> frozenset[str]:
        """把可能与完整车体包络碰撞的道路边保守封闭。"""

        blocked: set[str] = set()
        for obstacle in obstacles:
            if obstacle.radius_mm <= 0:
                continue
            required = self.clearance_mm + obstacle.radius_mm
            for edge in self.edges:
                start = self.nodes[edge.start]
                end = self.nodes[edge.end]
                if _point_segment_distance(obstacle, start, end) <= required:
                    blocked.add(edge.name)
        return frozenset(blocked)

    def route_is_blocked(
        self,
        plan: RoutePlan,
        waypoint_index: int,
        blocked_edges: frozenset[str],
    ) -> bool:
        """只检查尚未走完的路段；其他道路封闭或重新开放不打断当前路线。"""
        remaining = plan.nodes[max(0, waypoint_index - 1):]
        segments = {frozenset(pair) for pair in zip(remaining, remaining[1:])}
        return any(
            edge.name in blocked_edges and frozenset((edge.start, edge.end)) in segments
            for edge in self.edges
        )

    def plan(
        self,
        start: str,
        goal: str,
        *,
        blocked_edges: Iterable[str] = (),
    ) -> RoutePlan:
        if start not in self.nodes or goal not in self.nodes:
            raise ValueError("起点或终点节点不存在")
        blocked = frozenset(blocked_edges)
        adjacency: Dict[str, list[tuple[str, RoadEdge, float]]] = {
            name: [] for name in self.nodes
        }
        for edge in self.edges:
            if edge.name in blocked:
                continue
            if edge.width_mm / 2.0 < self.clearance_mm:
                continue
            distance = _distance(self.nodes[edge.start], self.nodes[edge.end])
            adjacency[edge.start].append((edge.end, edge, distance))
            adjacency[edge.end].append((edge.start, edge, distance))

        start_state: tuple[Optional[str], str] = (None, start)
        queue: list[tuple[float, float, int, tuple[Optional[str], str]]] = []
        serial = 0
        heapq.heappush(queue, (self._heuristic(start, goal), 0.0, serial, start_state))
        costs = {start_state: 0.0}
        parents: Dict[
            tuple[Optional[str], str], tuple[Optional[str], str]
        ] = {}
        goal_state: Optional[tuple[Optional[str], str]] = None

        while queue:
            _, cost, _, state = heapq.heappop(queue)
            previous, current = state
            if cost != costs.get(state):
                continue
            if current == goal:
                goal_state = state
                break
            for neighbor, _edge, length in adjacency[current]:
                penalty = self._turn_penalty(previous, current, neighbor)
                next_state = (current, neighbor)
                next_cost = cost + length + penalty
                if next_cost >= costs.get(next_state, math.inf):
                    continue
                costs[next_state] = next_cost
                parents[next_state] = state
                serial += 1
                heapq.heappush(
                    queue,
                    (
                        next_cost + self._heuristic(neighbor, goal),
                        next_cost,
                        serial,
                        next_state,
                    ),
                )

        if goal_state is None:
            raise NoRouteError(f"{start} 到 {goal} 没有可用路线")
        states = [goal_state]
        while states[-1] != start_state:
            states.append(parents[states[-1]])
        states.reverse()
        node_names = [start]
        node_names.extend(state[1] for state in states[1:])
        geometric_length = sum(
            _distance(self.nodes[a], self.nodes[b])
            for a, b in zip(node_names, node_names[1:])
        )
        return RoutePlan(tuple(node_names), blocked, geometric_length)

    def _heuristic(self, node: str, goal: str) -> float:
        return _distance(self.nodes[node], self.nodes[goal])

    def _turn_penalty(
        self,
        previous: Optional[str],
        current: str,
        following: str,
    ) -> float:
        if previous is None:
            return 0.0
        a, b, c = self.nodes[previous], self.nodes[current], self.nodes[following]
        first = math.atan2(b.y_mm - a.y_mm, b.x_mm - a.x_mm)
        second = math.atan2(c.y_mm - b.y_mm, c.x_mm - b.x_mm)
        turn = abs(_wrap_angle(second - first))
        return self.turn_penalty_mm * turn / math.pi


def _distance(first: Point2D, second: Point2D) -> float:
    return math.hypot(first.x_mm - second.x_mm, first.y_mm - second.y_mm)


def _point_segment_distance(point: Point2D, start: Point2D, end: Point2D) -> float:
    dx = end.x_mm - start.x_mm
    dy = end.y_mm - start.y_mm
    length_squared = dx * dx + dy * dy
    if length_squared == 0:
        return _distance(point, start)
    projection = (
        (point.x_mm - start.x_mm) * dx + (point.y_mm - start.y_mm) * dy
    ) / length_squared
    projection = max(0.0, min(1.0, projection))
    closest = Point2D(start.x_mm + projection * dx, start.y_mm + projection * dy)
    return _distance(point, closest)


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
