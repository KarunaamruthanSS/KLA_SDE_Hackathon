# Heuristic TSP Solver with Forbidden Zones

## 🔹 Purpose
This code solves a **Traveling Salesman Problem (TSP)** variant where:

- A stage (moving platform) must visit all dies (points) on a wafer.  
- Motion is constrained by **velocity and acceleration limits**.  
- Certain **forbidden rectangular zones** must be avoided.  
- Camera rotation (angular motion) may be required to align with die orientations.  

The solver outputs an **optimal visiting order of dies** and a **safe path** that avoids forbidden zones.

---

## 📂 Modules & Classes

### 1. `Point`
Represents a 2D coordinate with helper methods.

**Attributes:**
- `x`, `y`: Coordinates.

**Methods:**
- `from_tuple(t)`: Create from `(x, y)` tuple.  
- `to_list()`: Return `[x, y]` rounded for JSON output.  
- `distance_to(other)`: Euclidean distance to another point.  
- `__eq__`, `__hash__`: Equality and hashing for set operations.  
- `__repr__`: String representation.  

---

### 2. Motion Functions
- `motion_time(dist, vmax, amax)`:  
  Computes time to move a distance `dist` with trapezoidal/triangular velocity profile.  

- `segment_intersects_rect(p1, p2, bl, tr)`:  
  Checks if a line segment intersects a forbidden rectangle.  

- `project_to_rect_edge(p, bl, tr)`:  
  Projects a point inside a forbidden zone to the nearest edge.  

---

### 3. `TSPInputLoader`
Loads and preprocesses JSON input.

**Responsibilities:**
- Parse initial position, velocities, accelerations, angles.  
- Extract die centers and orientations.  
- Filter dies outside wafer radius.  
- Load forbidden zones.  

**Returns:**
- Die centers, orientations, initial position, motion parameters, forbidden zones.  

---

### 4. `HeuristicTSPSolver`
Core solver implementing obstacle-aware nearest-neighbor heuristic.

**Initialization:**
- Builds visibility graph with points and obstacle corners.  
- Computes safe paths using Floyd–Warshall (excluding dies as intermediates).  
- Precomputes stage times between dies.  

**Key Methods:**
- `_segment_is_safe(p1, p2)`: Check if direct path avoids forbidden zones.  
- `_get_safe_path_and_distance(from_idx, to_idx)`: Get shortest safe path.  
- `_move_cost(from_idx, to_idx, current_angle)`: Compute motion cost including angular rotation.  
- `nearest_neighbor()`: Construct path using heuristic:  
  - Prefer directly reachable dies.  
  - Use detours only when necessary.  
- `_path_cost(path)`: Compute total motion time.  
- `solve(restarts=20)`: Run solver with restarts, return best path and time.  

---

### 5. `TSPRunner`
Top-level runner that executes the solver.

**Responsibilities:**
- Load input JSON.  
- Run solver.  
- Build final path with intermediate waypoints only when needed.  
- Save output JSON.  

**Output Example:**
```json
{
  "TotalTime": 123.456,
  "Path": [[x0,y0],[x1,y1],...]
}
```

**`🧩 Workflow`**

1. **Input JSON is loaded** (`TSPInputLoader`).  
2. **Points & forbidden zones are processed.**  
3. **Visibility graph** is built with obstacle corners.  
4. **Stage times** are precomputed.  
5. **Nearest-neighbor heuristic** finds a valid die visiting order.  
6. **Final path** is constructed with detours only when necessary.  
7. **Output JSON** is written with total time and path.  
