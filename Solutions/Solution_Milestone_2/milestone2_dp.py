import json
from typing import List, Tuple
import math


class Point:
    """Simple container for 2D point operations."""
    
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
    
    @classmethod
    def from_tuple(cls, t: Tuple[float, float]):
        return cls(t[0], t[1])
    
    def to_list(self) -> List[float]:
        return [round(self.x, 3), round(self.y, 3)]  # Round to 3 decimals for output consistency
    
    def distance_to(self, other: 'Point') -> float:
        """Euclidean distance."""
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5


class TSPInputLoader:
    """Handles loading and parsing the input JSON data."""
    
    @staticmethod
    def load(json_data: dict) -> Tuple[List[Point], List[float], Point, float, float | None, float, float, float]:
        """
        Returns: die_centers, die_orients, initial_pos, stage_velocity, camera_velocity (None if not present),
                 stage_accel, camera_accel, initial_angle
        """
        initial_pos = Point.from_tuple(tuple(json_data["InitialPosition"]))
        stage_velocity = json_data["StageVelocity"]
        
        # Optional fields
        camera_velocity = json_data.get("CameraVelocity")
        stage_accel = json_data.get("StageAcceleration", 0.0)
        camera_accel = json_data.get("CameraAcceleration", 0.0)
        initial_angle = json_data.get("InitialAngle", 0.0)

        die_centers: List[Point] = []
        die_orients: List[float] = []
        for die in json_data["Dies"]:
            corners = die["Corners"]
            x_coords = [c[0] for c in corners]
            y_coords = [c[1] for c in corners]
            center_x = sum(x_coords) / 4.0
            center_y = sum(y_coords) / 4.0
            die_centers.append(Point(center_x, center_y))
            
            # Compute orientation using first two corners
            if len(corners) >= 2:
                x1, y1 = corners[0]
                x2, y2 = corners[1]
                dx = x2 - x1
                dy = y2 - y1
                orient = math.atan2(dy, dx) * 180 / math.pi
                die_orients.append(orient)

        return die_centers, die_orients, initial_pos, stage_velocity, camera_velocity, stage_accel, camera_accel, initial_angle


def motion_time(dist: float, vmax: float, amax: float) -> float:
    """Compute time for motion with max velocity and acceleration (trapezoidal profile). 
    Assumes deceleration = -amax."""
    if dist <= 0:
        return 0.0
    if amax <= 0:
        return dist / vmax if vmax > 0 else float('inf')
    # Time to reach vmax: t_acc = vmax / amax
    # Dist during acc: d_acc = 0.5 * amax * t_acc**2 = vmax**2 / (2 * amax)
    d_acc_dec = vmax ** 2 / amax  # acc + dec dist = 2 * d_acc
    if dist <= d_acc_dec:
        return 2 * math.sqrt(dist / amax)  # No constant vel phase
    else:
        t_acc_dec = 2 * (vmax / amax)
        d_const = dist - d_acc_dec
        t_const = d_const / vmax
        return t_acc_dec + t_const


class OpenTSPSolver:
    """Solves the open TSP using bitmask DP. Supports orientation choices for angular movement."""
    
    def __init__(self, points: List[Point], die_orients: List[float], stage_velocity: float, camera_velocity: float | None,
                 stage_accel: float, camera_accel: float, initial_angle: float):
        self.points = points  # points[0] is start, points[1:] are dies
        self.die_orients = die_orients  # len n
        self.stage_velocity = stage_velocity
        self.camera_velocity = camera_velocity
        self.stage_accel = stage_accel
        self.camera_accel = camera_accel
        self.initial_angle = initial_angle % 360
        self.use_angular = camera_velocity is not None
        self.n = len(points) - 1  # number of dies
    
    def _shortest_angular_dist(self, th1: float, th2: float) -> float:
        delta = abs(th1 - th2) % 360
        return min(delta, 360 - delta)
    
    def _solve_dp(self) -> Tuple[List[int], float]:
        if self.n == 0:
            return [], 0.0
        
        N = 1 << self.n
        ORIENTS = 4  # 0,90,180,270
        
        if not self.use_angular:
            # Standard DP without angular (Milestone 1)
            dp = [[float('inf')] * self.n for _ in range(N)]
            prev = [[(-1, -1)] * self.n for _ in range(N)]  # (prev_u, prev_ku) but ku unused
            # Base
            for j in range(self.n):
                dist = self.points[0].distance_to(self.points[j + 1])
                time = motion_time(dist, self.stage_velocity, self.stage_accel)
                dp[1 << j][j] = time
            # Fill
            for mask in range(N):
                for u in range(self.n):
                    if dp[mask][u] == float('inf'):
                        continue
                    for v in range(self.n):
                        if mask & (1 << v):
                            continue
                        new_mask = mask | (1 << v)
                        dist = self.points[u + 1].distance_to(self.points[v + 1])
                        t_stage = motion_time(dist, self.stage_velocity, self.stage_accel)
                        cost = dp[mask][u] + t_stage  # No angular
                        if cost < dp[new_mask][v]:
                            dp[new_mask][v] = cost
                            prev[new_mask][v] = (u, 0)  # dummy ku
            # Find min
            full_mask = N - 1
            min_time = float('inf')
            last = -1
            last_k = 0
            for j in range(self.n):
                if dp[full_mask][j] < min_time:
                    min_time = dp[full_mask][j]
                    last = j
            # Reconstruct
            path_indices = []
            current_mask = full_mask
            current_die = last
            current_k = last_k
            while current_die != -1:
                path_indices.append(current_die)
                prev_die, prev_k = prev[current_mask][current_die]
                if prev_die == -1:
                    break
                current_mask ^= (1 << current_die)
                current_die = prev_die
                current_k = prev_k
            path_indices.reverse()
            return path_indices, min_time
        
        else:
            # DP with orientations (Milestone 2/3)
            dp = [[[float('inf')] * ORIENTS for _ in range(self.n)] for _ in range(N)]
            prev = [[ [(-1, -1) for _ in range(ORIENTS)] for _ in range(self.n)] for _ in range(N)]
            # Base: from start to each j, each kj
            for j in range(self.n):
                dist = self.points[0].distance_to(self.points[j + 1])
                t_stage = motion_time(dist, self.stage_velocity, self.stage_accel)
                mask = 1 << j
                for kj in range(ORIENTS):
                    target_th = (self.die_orients[j] + kj * 90) % 360
                    delta = self._shortest_angular_dist(self.initial_angle, target_th)
                    t_ang = motion_time(delta, self.camera_velocity, self.camera_accel)
                    time = max(t_stage, t_ang)
                    dp[mask][j][kj] = time
                    prev[mask][j][kj] = (-1, -1)
            # Fill DP
            for mask in range(N):
                for u in range(self.n):
                    for ku in range(ORIENTS):
                        if dp[mask][u][ku] == float('inf'):
                            continue
                        for v in range(self.n):
                            if mask & (1 << v):
                                continue
                            new_mask = mask | (1 << v)
                            dist = self.points[u + 1].distance_to(self.points[v + 1])
                            t_stage = motion_time(dist, self.stage_velocity, self.stage_accel)
                            current_th = (self.die_orients[u] + ku * 90) % 360
                            for kv in range(ORIENTS):
                                target_th = (self.die_orients[v] + kv * 90) % 360
                                delta = self._shortest_angular_dist(current_th, target_th)
                                t_ang = motion_time(delta, self.camera_velocity, self.camera_accel)
                                add_cost = max(t_stage, t_ang)
                                cost = dp[mask][u][ku] + add_cost
                                if cost < dp[new_mask][v][kv]:
                                    dp[new_mask][v][kv] = cost
                                    prev[new_mask][v][kv] = (u, ku)
            # Find best ending
            full_mask = N - 1
            min_time = float('inf')
            last_die = -1
            last_k = -1
            for j in range(self.n):
                for k in range(ORIENTS):
                    if dp[full_mask][j][k] < min_time:
                        min_time = dp[full_mask][j][k]
                        last_die = j
                        last_k = k
            # Reconstruct path (only dies)
            path_indices = []
            current_mask = full_mask
            current_die = last_die
            current_k = last_k
            while current_die != -1:
                path_indices.append(current_die)
                prev_die, prev_k = prev[current_mask][current_die][current_k]
                if prev_die == -1:
                    break
                current_mask ^= (1 << current_die)
                current_die = prev_die
                current_k = prev_k
            path_indices.reverse()
            return path_indices, min_time
    
    def solve(self) -> Tuple[List[int], float]:
        return self._solve_dp()


class TSPRunner:
    """Orchestrates the full process: load → solve → output."""
    
    def __init__(self, input_file: str, output_file: str):
        self.input_file = input_file
        self.output_file = output_file
    
    def run(self):
        # Load data
        with open(self.input_file, 'r') as f:
            data = json.load(f)
        
        die_centers, die_orients, initial_pos, stage_velocity, camera_velocity, stage_accel, camera_accel, initial_angle = TSPInputLoader.load(data)
        
        # Prepare points list: start + dies
        points = [initial_pos] + die_centers
        
        # Solve using appropriate movement model
        solver = OpenTSPSolver(points, die_orients, stage_velocity, camera_velocity, stage_accel, camera_accel, initial_angle)
        optimal_die_order, total_time = solver.solve()
        
        # Build path coordinates
        path_coordinates = []
        path_coordinates.append(initial_pos.to_list())
        for die_idx in optimal_die_order:
            path_coordinates.append(die_centers[die_idx].to_list())
        
        # Prepare output
        output_dict = {
            "TotalTime": round(total_time, 3),
            "Path": path_coordinates
        }
        
        # Write output
        with open(self.output_file, 'w') as json_output:
            json.dump(output_dict, json_output, separators=(',', ':'))
        
        # Print for verification
        print(json.dumps(output_dict, separators=(',', ':')))


if __name__ == "__main__":
    # Update file paths as needed
    runner = TSPRunner(
        input_file='./TestCases/Milestone2/Input_milestone2_Testcase4.json',
        output_file='./TestCases/Milestone2/TestCase_2_4.json'
    )
    runner.run()
