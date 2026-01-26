import json
from typing import List, Tuple
import heapq


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
    """Solves the open TSP using Branch and Bound approach."""
    
    def __init__(self, points: List[Point], velocity: float):
        self.points = points  # points[0] is start, points[1:] are dies
        self.velocity = velocity
        self.n = len(points) - 1  # number of dies
    
    def _lower_bound(self, visited: List[bool], current_idx: int) -> float:
        """Estimate optimistic lower bound for remaining path cost."""
        bound = 0.0
        # Add minimum outgoing edge for each unvisited die
        for i in range(1, self.n + 1):
            if not visited[i - 1]:
                min_edge = float('inf')
                for j in range(1, self.n + 1):
                    if i != j:
                        min_edge = min(min_edge, self.points[i].distance_to(self.points[j]))
                bound += min_edge / self.velocity
        return bound
    
    def solve(self) -> Tuple[List[int], float]:
        if self.n == 0:
            return [], 0.0
        
        # Priority queue for branch and bound (min-heap)
        pq = []
        initial_state = (0.0, [0], [False] * self.n)  # (cost, path, visited dies)
        heapq.heappush(pq, initial_state)
        
        best_cost = float('inf')
        best_path = []
        
        while pq:
            cost, path, visited = heapq.heappop(pq)
            
            if len(path) == self.n + 1:  # visited all dies
                if cost < best_cost:
                    best_cost = cost
                    best_path = path[1:]  # exclude start index
                continue
            
            current_idx = path[-1]
            for die_idx in range(self.n):
                if not visited[die_idx]:
                    next_idx = die_idx + 1
                    new_cost = cost + self.points[current_idx].distance_to(self.points[next_idx]) / self.velocity
                    if new_cost >= best_cost:
                        continue
                    new_path = path + [next_idx]
                    new_visited = visited[:]
                    new_visited[die_idx] = True
                    bound = self._lower_bound(new_visited, next_idx)
                    if new_cost + bound < best_cost:
                        heapq.heappush(pq, (new_cost, new_path, new_visited))
        
        return [idx - 1 for idx in best_path], best_cost


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
