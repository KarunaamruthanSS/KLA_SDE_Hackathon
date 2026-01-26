import json
import math
from typing import List, Tuple

class Point:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def distance_to(self, other: 'Point') -> float:
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    def to_list(self):
        return [round(self.x, 3), round(self.y, 3)]


def motion_time(dist: float, vmax: float, amax: float) -> float:
    if dist <= 0:
        return 0.0
    if amax <= 0:
        return dist / vmax if vmax > 0 else float('inf')

    d_acc_dec = vmax**2 / amax
    if dist <= d_acc_dec:
        return 2 * math.sqrt(dist / amax)
    else:
        return 2 * (vmax / amax) + (dist - d_acc_dec) / vmax


def shortest_angular_dist(a1: float, a2: float) -> float:
    d = abs(a1 - a2) % 360
    return min(d, 360 - d)


def load_die_centers(data: dict):
    centers = []
    orients = []

    for die in data["Dies"]:
        corners = die["Corners"]
        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]

        center = Point(sum(xs)/4.0, sum(ys)/4.0)
        centers.append(center)

        dx = corners[1][0] - corners[0][0]
        dy = corners[1][1] - corners[0][1]
        orient = math.degrees(math.atan2(dy, dx)) % 360
        orients.append(orient)

    return centers, orients


def traverse_and_generate_path(json_file: str, output_file: str):
    with open(json_file, "r") as f:
        data = json.load(f)

    stage_v = data["StageVelocity"]
    stage_a = data.get("StageAcceleration", 0.0)
    cam_v = data.get("CameraVelocity", 0.0)
    cam_a = data.get("CameraAcceleration", 0.0)
    angle = data.get("InitialAngle", 0.0)

    init = Point(*data["InitialPosition"])
    centers, orients = load_die_centers(data)

    total_time = 0.0
    current_pos = init
    current_angle = angle

    path_coords = [init.to_list()]  # Start at initial position

    for i, center in enumerate(centers):
        # Stage motion
        dist = current_pos.distance_to(center)
        t_stage = motion_time(dist, stage_v, stage_a)

        # Camera motion (best of 4 orientations)
        base = orients[i]
        best_t = float("inf")
        best_angle = current_angle

        for k in range(4):
            target = (base + 90*k) % 360
            ang_dist = shortest_angular_dist(current_angle, target)
            t_ang = motion_time(ang_dist, cam_v, cam_a)
            cost = max(t_stage, t_ang)

            if cost < best_t:
                best_t = cost
                best_angle = target

        total_time += best_t
        current_pos = center
        current_angle = best_angle

        path_coords.append(center.to_list())

    output = {
        "TotalTime": round(total_time, 3),
        "Path": path_coords
    }

    with open(output_file, "w") as f:
        json.dump(output, f, separators=(",", ":"))

    print(json.dumps(output, separators=(",", ":")))
    print(f"\nTraversal complete. Total time = {total_time:.3f} seconds")


# Example usage:
traverse_and_generate_path(
     "./TestCases/Milestone3/Input_Milestone3_Testcase4.json",
     "./TestCases/Milestone3/TestCase_3_4.json"
)
