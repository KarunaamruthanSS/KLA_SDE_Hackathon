import json
import math
import random
from typing import List, Tuple, Set, Optional


class Point:
    """
    Simple 2D point with helper methods for distance and representation.
    """
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
    
    @classmethod
    def from_tuple(cls, t: Tuple[float, float]):
        return cls(t[0], t[1])
    
    def to_list(self) -> List[float]:
        """
        Return [x, y] rounded for cleaner JSON output.
        """
        return [round(self.x, 3), round(self.y, 3)]
    
    def distance_to(self, other: 'Point') -> float:
        """
        Euclidean distance to another point.
        """
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def __eq__(self, other):
        if not isinstance(other, Point):
            return False
        return math.isclose(self.x, other.x) and math.isclose(self.y, other.y)
    
    def __hash__(self):
        # Hash on rounded coords so we can store Points in sets.
        return hash((round(self.x, 6), round(self.y, 6)))
    
    def __repr__(self):
        return f"Point({self.x:.3f}, {self.y:.3f})"


def motion_time(dist: float, vmax: float, amax: float) -> float:
    """
    Time to move 'dist' with max velocity 'vmax' and acceleration 'amax'
    using a trapezoidal / triangular velocity profile.
    """
    if dist <= 0:
        return 0.0
    if amax <= 0:
        return dist / vmax if vmax > 0 else float('inf')
    d_acc_dec = vmax ** 2 / amax
    if dist <= d_acc_dec:
        # Triangular profile (never hits vmax)
        return 2 * math.sqrt(dist / amax)
    # Trapezoidal profile (accelerate, cruise, decelerate)
    return 2 * (vmax / amax) + (dist - d_acc_dec) / vmax


def segment_intersects_rect(p1: Point, p2: Point, bl: Tuple[float, float], tr: Tuple[float, float]) -> bool:
    """
    Check if the line segment p1->p2 intersects the interior of the axis-aligned rectangle
    defined by bottom-left (bl) and top-right (tr).

    We count it as intersection if:
    - either endpoint is strictly inside the rectangle, OR
    - the segment intersects any of the 4 edges.

    We deliberately do NOT use a midpoint heuristic here, because that can
    falsely classify segments that pass near the rectangle as intersecting.
    """
    x1, y1 = p1.x, p1.y
    x2, y2 = p2.x, p2.y
    rx1, ry1 = bl
    rx2, ry2 = tr

    # Endpoint strictly inside
    if rx1 < x1 < rx2 and ry1 < y1 < ry2:
        return True
    if rx1 < x2 < rx2 and ry1 < y2 < ry2:
        return True

    # Rectangle edges as segments
    edges = [
        ((rx1, ry1), (rx2, ry1)),  # bottom
        ((rx2, ry1), (rx2, ry2)),  # right
        ((rx2, ry2), (rx1, ry2)),  # top
        ((rx1, ry2), (rx1, ry1)),  # left
    ]

    def ccw(A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    def intersect(A, B, C, D):
        return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)

    A = (x1, y1)
    B = (x2, y2)

    # Check if segment intersects any edge
    for C, D in edges:
        if intersect(A, B, C, D):
            return True

    # If no edge intersection and no endpoint inside, treat as safe.
    return False


def project_to_rect_edge(p: Point, bl: Tuple[float, float], tr: Tuple[float, float]) -> Point:
    """
    Project point p onto the nearest edge of the rectangle defined by (bl, tr).
    Used when we must 'bounce' off the forbidden zone boundary without entering it.
    """
    x, y = p.x, p.y
    rx1, ry1 = bl
    rx2, ry2 = tr

    # Inside horizontally: snap vertically to nearest top/bottom edge
    if rx1 <= x <= rx2:
        return Point(x, ry1 if abs(y - ry1) < abs(y - ry2) else ry2)

    # Inside vertically: snap horizontally to nearest left/right edge
    if ry1 <= y <= ry2:
        return Point(rx1 if abs(x - rx1) < abs(x - rx2) else rx2, y)

    # Outside in both directions: snap to nearest corner
    px = rx1 if abs(x - rx1) < abs(x - rx2) else rx2
    py = ry1 if abs(y - ry1) < abs(y - ry2) else ry2
    return Point(px, py)


class TSPInputLoader:
    """
    Responsible for reading the JSON input and converting to:
    - initial position
    - die centers and orientations
    - motion parameters
    - forbidden zones
    """
    @staticmethod
    def load(json_data: dict):
        initial_pos = Point.from_tuple(tuple(json_data["InitialPosition"]))
        stage_velocity = json_data["StageVelocity"]
        camera_velocity = json_data.get("CameraVelocity")
        stage_accel = json_data.get("StageAcceleration", 0.0)
        camera_accel = json_data.get("CameraAcceleration", 0.0)
        initial_angle = json_data.get("InitialAngle", 0.0)

        wafer_diameter = json_data.get("WaferDiameter", None)
        wafer_radius = wafer_diameter / 2 if wafer_diameter else None

        die_centers: List[Point] = []
        die_orients: List[float] = []

        for die in json_data["Dies"]:
            corners = die["Corners"]
            xs = [c[0] for c in corners]
            ys = [c[1] for c in corners]
            center = Point(sum(xs)/4, sum(ys)/4)

            # Filter dies using wafer radius: keep if any corner is inside the wafer.
            if wafer_radius is not None:
                if not any(math.hypot(x, y) <= wafer_radius for x, y in corners):
                    continue
            die_centers.append(center)

            # Orientation: use vector from corner0 to corner1 as base orientation
            if len(corners) >= 2:
                dx = corners[1][0] - corners[0][0]
                dy = corners[1][1] - corners[0][1]
                die_orients.append(math.degrees(math.atan2(dy, dx)) % 360)
            else:
                die_orients.append(0.0)

        # Forbidden zones as list of (BottomLeft, TopRight)
        forbidden_zones = [
            (tuple(r["BottomLeft"]), tuple(r["TopRight"]))
            for r in json_data.get("ForbiddenZones", [])
        ]

        return (
            die_centers,
            die_orients,
            initial_pos,
            stage_velocity,
            camera_velocity,
            stage_accel,
            camera_accel,
            initial_angle,
            forbidden_zones,
        )


class HeuristicTSPSolver:
    """
    Main solver:
    - Builds a visibility-like graph including corner nodes
    - Precomputes obstacle-aware stage times between initial + dies
    - Uses a nearest-neighbor heuristic with an important rule:
        At each step, if any unvisited die is reachable by a straight line
        (no forbidden zone intersection), pick the nearest such die.
        Only when all remaining dies require detours, use the corner graph.
    """

    def __init__(self, points, die_orients, stage_velocity, camera_velocity,
                 stage_accel, camera_accel, initial_angle, forbidden_zones):
        # points[0] = initial position; points[1..n] = die centers
        self.points = points
        self.die_orients = die_orients
        self.stage_velocity = stage_velocity
        self.camera_velocity = camera_velocity
        self.stage_accel = stage_accel
        self.camera_accel = camera_accel
        self.initial_angle = initial_angle % 360
        self.use_angular = camera_velocity is not None and camera_velocity > 0
        self.forbidden_zones = forbidden_zones
        self.n = len(points) - 1  # number of dies

        # If any point is inside a forbidden zone, project it to the nearest edge.
        for i in range(len(self.points)):
            if self._is_point_in_obstacle(self.points[i]):
                for bl, tr in forbidden_zones:
                    if bl[0] < self.points[i].x < tr[0] and bl[1] < self.points[i].y < tr[1]:
                        self.points[i] = project_to_rect_edge(self.points[i], bl, tr)
                        break

        # Build the global node list: all stage points + all obstacle corners.
        self.all_nodes = self.points[:]
        corner_set = set()
        for bl, tr in forbidden_zones:
            for c in [
                Point(bl[0], bl[1]),
                Point(tr[0], bl[1]),
                Point(tr[0], tr[1]),
                Point(bl[0], tr[1])
            ]:
                corner_set.add(c)
        self.all_nodes.extend(list(corner_set))

        self.v = len(self.all_nodes)
        inf = float('inf')

        # Graph distances between all_nodes[i] and all_nodes[j]
        self.graph_dist = [[inf] * self.v for _ in range(self.v)]
        for i in range(self.v):
            self.graph_dist[i][i] = 0.0

        # Add direct visible edges; if blocked, connect via corners.
        for i in range(self.v):
            for j in range(i + 1, self.v):
                p1, p2 = self.all_nodes[i], self.all_nodes[j]
                if self._segment_is_safe(p1, p2):
                    # Straight segment between these nodes is safe.
                    d = p1.distance_to(p2)
                    self.graph_dist[i][j] = self.graph_dist[j][i] = d
                else:
                    # Blocked: connect to corners of blocking rectangles (if visible).
                    for bl, tr in forbidden_zones:
                        if segment_intersects_rect(p1, p2, bl, tr):
                            for cx, cy in [
                                (bl[0], bl[1]),
                                (tr[0], bl[1]),
                                (tr[0], tr[1]),
                                (bl[0], tr[1])
                            ]:
                                c = Point(cx, cy)
                                idx = self.all_nodes.index(c)
                                if self._segment_is_safe(p1, c):
                                    self.graph_dist[i][idx] = min(
                                        self.graph_dist[i][idx], p1.distance_to(c)
                                    )
                                if self._segment_is_safe(p2, c):
                                    self.graph_dist[j][idx] = min(
                                        self.graph_dist[j][idx], p2.distance_to(c)
                                    )

        # Floyd–Warshall on graph_dist, but do NOT use die nodes (1..n) as intermediates,
        # so dies are never used as relay points between other nodes.
        self.next_node = [[-1] * self.v for _ in range(self.v)]
        for i in range(self.v):
            for j in range(self.v):
                if self.graph_dist[i][j] < inf and i != j:
                    self.next_node[i][j] = j

        for k in range(self.v):
            # Skip die nodes as intermediate nodes
            if 1 <= k <= self.n:
                continue
            for i in range(self.v):
                for j in range(self.v):
                    if self.graph_dist[i][k] + self.graph_dist[k][j] < self.graph_dist[i][j]:
                        self.graph_dist[i][j] = self.graph_dist[i][k] + self.graph_dist[k][j]
                        self.next_node[i][j] = self.next_node[i][k]

        # Precompute stage times between initial + dies using obstacle-aware paths.
        # BUT: if a direct straight segment between points[i] and points[j] is safe,
        # we use that direct time (no detour).
        self.stage_time = [[0.0] * (self.n + 1) for _ in range(self.n + 1)]
        for i in range(self.n + 1):
            for j in range(self.n + 1):
                if i == j:
                    continue
                p1 = self.points[i]
                p2 = self.points[j]
                if self._segment_is_safe(p1, p2):
                    # Direct safe segment
                    dist = p1.distance_to(p2)
                    self.stage_time[i][j] = motion_time(dist, self.stage_velocity, self.stage_accel)
                else:
                    # Need a detour in the visibility graph
                    dist, path = self._get_safe_path_and_distance(i, j)
                    t = 0.0
                    for k in range(len(path) - 1):
                        t += motion_time(
                            path[k].distance_to(path[k + 1]),
                            self.stage_velocity,
                            self.stage_accel
                        )
                    self.stage_time[i][j] = t

    def _shortest_angular_dist(self, th1, th2):
        """
        Minimal absolute angular difference between th1 and th2 in degrees.
        """
        delta = abs(th1 - th2) % 360
        return min(delta, 360 - delta)

    def _is_point_in_obstacle(self, p: Point) -> bool:
        """
        Check if a point lies strictly inside any forbidden zone.
        """
        for bl, tr in self.forbidden_zones:
            if bl[0] < p.x < tr[0] and bl[1] < p.y < tr[1]:
                return True
        return False

    def _segment_is_safe(self, p1: Point, p2: Point) -> bool:
        """
        Check if the straight segment p1->p2 avoids all forbidden zones.
        """
        for bl, tr in self.forbidden_zones:
            if segment_intersects_rect(p1, p2, bl, tr):
                return False
        return True

    def _get_safe_path_and_distance(self, from_idx: int, to_idx: int) -> Tuple[float, List[Point]]:
        """
        Return the shortest path (in the graph) between all_nodes[from_idx] and all_nodes[to_idx]:
        - If the direct segment is safe, return [start, end].
        - Otherwise, reconstruct path from Floyd–Warshall 'next_node' matrix.
        """
        p1 = self.all_nodes[from_idx]
        p2 = self.all_nodes[to_idx]

        # If direct segment is safe, no need for graph.
        if self._segment_is_safe(p1, p2):
            return p1.distance_to(p2), [p1, p2]

        if from_idx == to_idx:
            return 0.0, [p1]

        dist = self.graph_dist[from_idx][to_idx]
        if dist >= float('inf'):
            # Should be rare; fallback to straight segment as last resort
            return p1.distance_to(p2), [p1, p2]

        # Reconstruct path via next_node
        path_idx = []
        curr = from_idx
        while curr != to_idx:
            path_idx.append(curr)
            next_curr = self.next_node[curr][to_idx]
            if next_curr == -1:
                break
            curr = next_curr
        path_idx.append(to_idx)
        path = [self.all_nodes[k] for k in path_idx]
        return dist, path

    def _stage_time_with_obstacles(self, from_idx: int, to_idx: int) -> float:
        """
        Precomputed stage time between points[from_idx] and points[to_idx].
        """
        return self.stage_time[from_idx][to_idx]

    def _move_cost(self, from_idx: int, to_idx: int, current_angle: float) -> Tuple[float, float]:
        """
        Time to move from from_idx to to_idx, including:
        - Stage translation time (with forbidden zones).
        - Camera rotation time (if enabled).
        Returns (time_cost, new_camera_angle).
        """
        t_stage = self._stage_time_with_obstacles(from_idx, to_idx)
        if not self.use_angular:
            return t_stage, current_angle

        # to_idx >= 1 => die index is (to_idx - 1)
        base_orient = self.die_orients[to_idx - 1]
        best_cost, best_angle = float('inf'), current_angle

        # Camera can align at base_orient + k*90 degrees
        for k in range(4):
            target_th = (base_orient + k * 90) % 360
            ang_dist = self._shortest_angular_dist(current_angle, target_th)
            t_ang = motion_time(ang_dist, self.camera_velocity, self.camera_accel)
            cost = max(t_stage, t_ang)
            if cost < best_cost:
                best_cost = cost
                best_angle = target_th

        return best_cost, best_angle

    def nearest_neighbor(self) -> List[int]:
        """
        Build a path visiting all dies exactly once, using this rule:

        1. At each step, partition unvisited dies into:
           - direct_reachable: straight line from current position to die does NOT
             intersect any forbidden zone.
           - blocked: all others.

        2. If direct_reachable is non-empty:
           - Choose the die with minimal Euclidean distance from current position.
           - Move there (cost uses full motion model, but choice is distance-based).

        3. If direct_reachable is empty:
           - Compute move_cost for all blocked dies (including detours).
           - Choose the die with minimal move_cost.

        This enforces your priority:
        "Always go to a directly reachable die first; only use edge/corner detours
        when every remaining die would cross a forbidden region."
        """
        unvisited = set(range(self.n))  # dies are indexed 0..n-1
        path: List[int] = []
        current_idx = 0  # points index: 0 = initial position
        current_angle = self.initial_angle

        while unvisited:
            direct_reachable = []
            blocked = []

            # Split dies based on direct segment safety from current position.
            for die in unvisited:
                p1 = self.points[current_idx]
                p2 = self.points[die + 1]  # die i is at points[i+1]
                if self._segment_is_safe(p1, p2):
                    direct_reachable.append(die)
                else:
                    blocked.append(die)

            # CASE 1: Some dies are directly reachable.
            if direct_reachable:
                # Choose nearest in straight-line distance (fast heuristic).
                best_die = min(
                    direct_reachable,
                    key=lambda d: self.points[current_idx].distance_to(self.points[d + 1])
                )
                path.append(best_die)
                # Update angle using full motion model.
                _, current_angle = self._move_cost(current_idx, best_die + 1, current_angle)
                unvisited.remove(best_die)
                current_idx = best_die + 1
                continue

            # CASE 2: No die is directly reachable => must use detours.
            candidates = []
            for die in blocked:
                cost, _ = self._move_cost(current_idx, die + 1, current_angle)
                candidates.append((cost, die))

            candidates.sort(key=lambda x: x[0])
            best_cost, best_die = candidates[0]

            path.append(best_die)
            _, current_angle = self._move_cost(current_idx, best_die + 1, current_angle)
            unvisited.remove(best_die)
            current_idx = best_die + 1

        return path

    def _validate_path(self, path: List[int]) -> bool:
        """
        Sanity check: path must be a permutation of 0..n-1.
        """
        if len(path) != self.n:
            return False
        if len(set(path)) != self.n:
            return False
        if set(path) != set(range(self.n)):
            return False
        if any(p < 0 or p >= self.n for p in path):
            return False
        return True

    def _path_cost(self, path: List[int]) -> float:
        """
        Compute total motion cost for a given die order, including angular motion.
        """
        total = 0.0
        current_idx = 0
        current_angle = self.initial_angle
        for die in path:
            cost, current_angle = self._move_cost(current_idx, die + 1, current_angle)
            total += cost
            current_idx = die + 1
        return total

    def solve(self, restarts: int = 20) -> Tuple[List[int], float]:
        """
        Solve for a good path:
        - Use nearest_neighbor() to build an initial path.
        - Optionally repeat with restarts (you can add randomness if desired).
        - No 2-opt here, to preserve the "direct reachable first" rule.
        """
        best_path: Optional[List[int]] = None
        best_time = float('inf')

        for _ in range(restarts):
            path = self.nearest_neighbor()

            if not self._validate_path(path):
                # Fallback: if something goes wrong, just use sequential dies.
                path = list(range(self.n))

            time = self._path_cost(path)

            if time < best_time:
                best_time = time
                best_path = path

        if best_path is None:
            raise RuntimeError("No valid path found after all restarts.")

        # Final sanity check
        if not self._validate_path(best_path):
            raise RuntimeError("Final path invalid after NN.")

        return best_path, best_time


class TSPRunner:
    """
    Top-level runner:
    - Reads input JSON
    - Runs the solver
    - Builds the final Path with intermediate waypoints only when needed
      (i.e., only when direct segments would cross forbidden zones).
    """
    def __init__(self, input_file: str, output_file: str):
        self.input_file = input_file
        self.output_file = output_file

    def run(self):
        with open(self.input_file, 'r') as f:
            data = json.load(f)

        (
            die_centers,
            die_orients,
            initial_pos,
            stage_velocity,
            camera_velocity,
            stage_accel,
            camera_accel,
            initial_angle,
            forbidden_zones,
        ) = TSPInputLoader.load(data)

        # points[0] = initial; points[1..n] = die centers
        points = [initial_pos] + die_centers

        solver = HeuristicTSPSolver(
            points,
            die_orients,
            stage_velocity,
            camera_velocity,
            stage_accel,
            camera_accel,
            initial_angle,
            forbidden_zones,
        )

        random.seed(42)
        optimal_die_order, total_time = solver.solve(restarts=1)

        # Build final output path:
        # - If a segment (current → die) is safe, emit the die directly.
        # - If not, emit the intermediate waypoints from the visibility graph.
        path_coordinates = [initial_pos.to_list()]
        current_idx = 0
        for die_idx in optimal_die_order:
            p1 = points[current_idx]
            p2 = points[die_idx + 1]

            if solver._segment_is_safe(p1, p2):
                # Direct move: no intermediate waypoints in output.
                path_coordinates.append(p2.to_list())
            else:
                # Blocked: include detour waypoints from the graph.
                _, waypoints = solver._get_safe_path_and_distance(current_idx, die_idx + 1)
                for wp in waypoints[1:]:  # Skip first (already at p1)
                    path_coordinates.append(wp.to_list())

            current_idx = die_idx + 1

        output_dict = {
            "TotalTime": round(total_time, 3),
            "Path": path_coordinates
        }

        with open(self.output_file, 'w') as f:
            json.dump(output_dict, f, separators=(',', ':'))

        print(json.dumps(output_dict, separators=(',', ':')))
        print(f"\nSuccess! Path visits all {len(die_centers)} dies exactly once.")
        print(f"Total time: {total_time:.3f} s")
        print(f"Points in output: {len(path_coordinates)} (direct where possible)")


if __name__ == "__main__":
    runner = TSPRunner('./Input_Milestone4_Testcase4.json', './TestCase_4_4.json')
    runner.run()
