import json
import math
import random
from typing import List, Tuple

class Point:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
    
    @classmethod
    def from_tuple(cls, t: Tuple[float, float]):
        return cls(t[0], t[1])
    
    def to_list(self) -> List[float]:
        return [round(self.x, 3), round(self.y, 3)]
    
    def distance_to(self, other: 'Point') -> float:
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)


class TSPInputLoader:
    @staticmethod
    def load(json_data: dict) -> Tuple[List[Point], List[float], Point, float, float | None, float, float, float]:
        initial_pos = Point.from_tuple(tuple(json_data["InitialPosition"]))
        stage_velocity = json_data["StageVelocity"]
        camera_velocity = json_data.get("CameraVelocity")
        stage_accel = json_data.get("StageAcceleration", 0.0)
        camera_accel = json_data.get("CameraAcceleration", 0.0)
        initial_angle = json_data.get("InitialAngle", 0.0)

        die_centers: List[Point] = []
        die_orients: List[float] = []
        for die in json_data["Dies"]:
            corners = die["Corners"]
            xs = [c[0] for c in corners]
            ys = [c[1] for c in corners]
            center = Point(sum(xs)/4.0, sum(ys)/4.0)
            die_centers.append(center)
            
            if len(corners) >= 2:
                dx = corners[1][0] - corners[0][0]
                dy = corners[1][1] - corners[0][1]
                orient = math.degrees(math.atan2(dy, dx))
                die_orients.append(orient % 360)
            else:
                die_orients.append(0.0)

        return die_centers, die_orients, initial_pos, stage_velocity, camera_velocity, stage_accel, camera_accel, initial_angle


def motion_time(dist: float, vmax: float, amax: float) -> float:
    if dist <= 0:
        return 0.0
    if amax <= 0:
        return dist / vmax if vmax > 0 else float('inf')
    d_acc_dec = vmax ** 2 / amax
    if dist <= d_acc_dec:
        return 2 * math.sqrt(dist / amax)
    else:
        return 2 * (vmax / amax) + (dist - d_acc_dec) / vmax


class HeuristicTSPSolver:
    def __init__(self, points: List[Point], die_orients: List[float], 
                 stage_velocity: float, camera_velocity: float | None,
                 stage_accel: float, camera_accel: float, initial_angle: float):
        self.points = points
        self.die_orients = die_orients
        self.stage_velocity = stage_velocity
        self.camera_velocity = camera_velocity
        self.stage_accel = stage_accel
        self.camera_accel = camera_accel
        self.initial_angle = initial_angle % 360
        self.use_angular = camera_velocity is not None and camera_velocity > 0
        self.n = len(points) - 1

    def _shortest_angular_dist(self, th1: float, th2: float) -> float:
        delta = abs(th1 - th2) % 360
        return min(delta, 360 - delta)

    def _move_cost(self, from_idx: int, to_idx: int, current_angle: float) -> Tuple[float, float]:
        dist = self.points[from_idx].distance_to(self.points[to_idx])
        t_stage = motion_time(dist, self.stage_velocity, self.stage_accel)

        if not self.use_angular:
            return t_stage, current_angle

        base_orient = self.die_orients[to_idx - 1]
        best_cost = float('inf')
        best_angle = current_angle
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
        unvisited = set(range(self.n))
        path = []
        current_idx = 0
        current_angle = self.initial_angle

        while unvisited:
            best_die = -1
            best_cost = float('inf')
            for die in unvisited:
                cost, _ = self._move_cost(current_idx, die + 1, current_angle)
                if cost < best_cost:
                    best_cost = cost
                    best_die = die
            path.append(best_die)
            _, current_angle = self._move_cost(current_idx, best_die + 1, current_angle)
            unvisited.remove(best_die)
            current_idx = best_die + 1

        return path

    def _path_cost(self, path: List[int]) -> float:
        total = 0.0
        current_idx = 0
        current_angle = self.initial_angle
        for die in path:
            cost, current_angle = self._move_cost(current_idx, die + 1, current_angle)
            total += cost
            current_idx = die + 1
        return total

    def two_opt(self, path: List[int]) -> List[int]:
        best_path = path[:]
        best_cost = self._path_cost(best_path)
        improved = True

        while improved:
            improved = False
            for i in range(len(best_path) - 1):
                for j in range(i + 2, len(best_path)):
                    new_path = best_path[:i+1] + best_path[i+1:j+1][::-1] + best_path[j+1:]
                    new_cost = self._path_cost(new_path)
                    if new_cost < best_cost:
                        best_path = new_path
                        best_cost = new_cost
                        improved = True

        return best_path

    def solve(self, restarts: int = 20) -> Tuple[List[int], float]:
        best_path = None
        best_time = float('inf')

        for _ in range(restarts):
            path = self.nearest_neighbor()
            path = self.two_opt(path)
            time = self._path_cost(path)
            if time < best_time:
                best_time = time
                best_path = path

        return best_path, best_time


class TSPRunner:
    def __init__(self, input_file: str, output_file: str):
        self.input_file = input_file
        self.output_file = output_file
    
    def run(self):
        with open(self.input_file, 'r') as f:
            data = json.load(f)
        
        die_centers, die_orients, initial_pos, stage_velocity, camera_velocity, \
        stage_accel, camera_accel, initial_angle = TSPInputLoader.load(data)
        
        points = [initial_pos] + die_centers
        
        solver = HeuristicTSPSolver(
            points, die_orients, stage_velocity, camera_velocity,
            stage_accel, camera_accel, initial_angle
        )
        
        random.seed(42)
        optimal_die_order, total_time = solver.solve(restarts=20)
        
        path_coordinates = [initial_pos.to_list()]
        for die_idx in optimal_die_order:
            path_coordinates.append(die_centers[die_idx].to_list())
        
        output_dict = {
            "TotalTime": round(total_time, 3),
            "Path": path_coordinates
        }
        
        with open(self.output_file, 'w') as f:
            json.dump(output_dict, f, separators=(',', ':'))
        
        print(json.dumps(output_dict, separators=(',', ':')))


if __name__ == "__main__":
    # Update paths for your test case
    runner = TSPRunner(
        input_file='./TestCases/Milestone2/Input_Milestone2_Testcase3.json',
        output_file='./TestCases/Milestone2/TestCase_2_3.json'
    )
    runner.run()
