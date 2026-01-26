import json
from typing import List, Tuple


class Point:
    """Simple container for 2D point operations."""
    
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
    
    @classmethod
    def from_tuple(cls, t: Tuple[float, float]):
        return cls(t[0], t[1])
    
    def to_list(self) -> List[float]:
        return [self.x, self.y]
    
    def distance_to(self, other: 'Point') -> float:
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5


class TSPInputLoader:
    """Handles loading and parsing the input JSON data."""
    
    @staticmethod
    def load(json_data: dict) -> Tuple[List[Point], Point, float]:
        initial_pos = Point.from_tuple(tuple(json_data["InitialPosition"]))
        stage_velocity = json_data["StageVelocity"]

        die_centers: List[Point] = []
        for die in json_data["Dies"]:
            corners = die["Corners"]
            x_coords = [c[0] for c in corners]
            y_coords = [c[1] for c in corners]
            center_x = sum(x_coords) / 4.0
            center_y = sum(y_coords) / 4.0
            die_centers.append(Point(center_x, center_y))

        return die_centers, initial_pos, stage_velocity


class OpenTSPSolver:
    """Solves the open TSP using bitmask DP (exact same logic as original)."""
    
    def __init__(self, points: List[Point], velocity: float):
        self.points = points  # points[0] is start, points[1:] are dies
        self.velocity = velocity
        self.n = len(points) - 1  # number of dies
    
    def _solve_dp(self) -> Tuple[List[int], float]:
        if self.n == 0:
            return [], 0.0
        
        N = 1 << self.n
        dp = [[float('inf')] * self.n for _ in range(N)]
        prev = [[-1] * self.n for _ in range(N)]

        # Base case: from start to each die
        for j in range(self.n):
            dist = self.points[0].distance_to(self.points[j + 1])
            dp[1 << j][j] = dist / self.velocity

        # Fill DP table
        for mask in range(N):
            for u in range(self.n):
                if dp[mask][u] == float('inf'):
                    continue
                for v in range(self.n):
                    if mask & (1 << v):
                        continue
                    new_mask = mask | (1 << v)
                    cost = dp[mask][u] + self.points[u + 1].distance_to(self.points[v + 1]) / self.velocity
                    if cost < dp[new_mask][v]:
                        dp[new_mask][v] = cost
                        prev[new_mask][v] = u

        # Find best ending point
        full_mask = N - 1
        min_time = float('inf')
        last_die = -1
        for j in range(self.n):
            if dp[full_mask][j] < min_time:
                min_time = dp[full_mask][j]
                last_die = j

        # Reconstruct path
        path_indices = []
        current_mask = full_mask
        current_die = last_die
        while current_die != -1:
            path_indices.append(current_die)
            prev_die = prev[current_mask][current_die]
            if prev_die == -1:
                break
            current_mask ^= (1 << current_die)
            current_die = prev_die

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
        
        die_centers, initial_pos, velocity = TSPInputLoader.load(data)
        
        # Prepare points list: start + dies
        points = [initial_pos] + die_centers
        
        # Solve
        solver = OpenTSPSolver(points, velocity)
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
    runner = TSPRunner(
        input_file='./TestCases/Milestone1/Input_milestone1_Testcase4.json',
        output_file='./TestCases/Milestone1/TestCase_1_4.json'
    )
    runner.run()