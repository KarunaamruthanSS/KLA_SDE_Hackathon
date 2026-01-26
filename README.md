# Software Workshop 2025: Advanced TSP Solver for Semiconductor Manufacturing

This repository contains solutions for a progressive software workshop focused on solving increasingly complex variants of the Traveling Salesman Problem (TSP) in the context of semiconductor wafer inspection systems.

## 🎯 Workshop Overview

The workshop progresses through 4 milestones, each building upon the previous one to solve more realistic manufacturing scenarios:

### **Milestone 1: Basic Open TSP**
- **Problem**: Visit all dies on a wafer starting from an initial position
- **Constraints**: Simple velocity-based motion model
- **Solutions**: 
  - `milestone-1_dp.py`: Dynamic programming with bitmask (exact solution)
  - `milestone-1_bb.py`: Branch and bound approach (exact solution)
  - `milestone-1_c.py`: Alternative DP implementation

### **Milestone 2: Motion with Angular Constraints**
- **Problem**: Add camera rotation requirements for die alignment
- **New Features**: 
  - Camera velocity and acceleration limits
  - Die orientation alignment (0°, 90°, 180°, 270°)
  - Trapezoidal motion profiles for both stage and camera
- **Solutions**:
  - `milestone2_dp.py`: Exact DP solution with orientation states
  - `milestone2_heuristic.py`: Heuristic approach with 2-opt optimization

### **Milestone 3: Wafer Boundary Constraints**
- **Problem**: Filter dies based on wafer diameter
- **New Features**:
  - Wafer radius filtering (only visit dies within the wafer boundary)
  - Improved path validation and repair mechanisms
- **Solution**: `milestone3_heuristic.py`: Enhanced heuristic with wafer constraints

### **Milestone 4: Obstacle Avoidance**
- **Problem**: Navigate around forbidden rectangular zones
- **New Features**:
  - Forbidden zones that cannot be crossed
  - Visibility graph construction with obstacle corners
  - Floyd-Warshall shortest path computation
  - Intelligent path planning that prefers direct routes when safe
- **Solution**: `milestone_4.py`: Complete obstacle-aware TSP solver

## 🏗️ Architecture

### Core Components

#### **Point Class**
```python
class Point:
    def __init__(self, x: float, y: float)
    def distance_to(other: Point) -> float
    def to_list() -> List[float]  # For JSON output
```

#### **Motion Model**
```python
def motion_time(dist: float, vmax: float, amax: float) -> float
```
Implements trapezoidal/triangular velocity profiles for realistic motion planning.

#### **TSPInputLoader**
Parses JSON input containing:
- Initial stage position and camera angle
- Stage/camera velocity and acceleration limits
- Die corner coordinates (converted to centers and orientations)
- Wafer diameter for boundary filtering
- Forbidden rectangular zones

#### **Solver Classes**
- **OpenTSPSolver**: Exact solutions using DP or branch-and-bound
- **HeuristicTSPSolver**: Fast approximate solutions with local optimization

### Key Algorithms

#### **Dynamic Programming (Milestones 1-2)**
- Bitmask DP for exact TSP solutions
- State space: `dp[mask][last_die][orientation]`
- Time complexity: O(n² × 2ⁿ × 4) for angular version

#### **Heuristic Approach (Milestones 2-4)**
- Nearest neighbor construction with intelligent tie-breaking
- 2-opt local search optimization
- Multiple restarts for solution quality

#### **Obstacle Avoidance (Milestone 4)**
- Visibility graph with obstacle corner nodes
- Floyd-Warshall for shortest obstacle-free paths
- Priority system: direct paths preferred over detours

## 📁 Project Structure

```
├── Solutions/
│   ├── Solution_Milestone_1/     # Basic TSP solutions
│   ├── Solution_Milestone_2/     # Angular motion solutions  
│   ├── Solution_Milestone_3/     # Wafer boundary solutions
│   └── Solution_Milestone_4/     # Complete obstacle avoidance
├── TestCases/
│   ├── Milestone1/              # Test inputs for each milestone
│   ├── Milestone2/
│   ├── Milestone3/
│   └── Milestone4/
├── Software_Workshop_Presentation_2025_Day1.pdf
├── Software_Workshop_Presentation_2025_Day2.pdf
└── README.md
```

## 🚀 Usage

Each solution can be run independently:

```python
# Example: Run Milestone 4 solution
python Solutions/Solution_Milestone_4/milestone_4.py

# Input: JSON file with dies, constraints, and forbidden zones
# Output: Optimal path with total time
```

### Input Format
```json
{
  "InitialPosition": [-100, 0],
  "InitialAngle": 0,
  "StageVelocity": 50,
  "StageAcceleration": 100,
  "CameraVelocity": 10,
  "CameraAcceleration": 20,
  "WaferDiameter": 300,
  "Dies": [
    {
      "Corners": [[x1,y1], [x2,y2], [x3,y3], [x4,y4]]
    }
  ],
  "ForbiddenZones": [
    {
      "BottomLeft": [-30, -30],
      "TopRight": [30, 30]
    }
  ]
}
```

### Output Format
```json
{
  "TotalTime": 123.456,
  "Path": [[x0,y0], [x1,y1], [x2,y2], ...]
}
```

## 🎓 Learning Objectives

This workshop demonstrates:
- **Algorithm Design**: Progression from exact to heuristic approaches
- **Constraint Handling**: Adding realistic manufacturing constraints incrementally  
- **Optimization Techniques**: DP, branch-and-bound, local search, graph algorithms
- **Software Engineering**: Modular design, code reuse, and incremental development
- **Real-world Applications**: Semiconductor manufacturing, robotics, and motion planning

## 🔧 Technical Highlights

- **Exact Solutions**: Guaranteed optimal for small instances (≤15 dies)
- **Scalable Heuristics**: Handle hundreds of dies efficiently
- **Motion Realism**: Trapezoidal velocity profiles with acceleration limits
- **Obstacle Handling**: Visibility graphs for complex geometric constraints
- **Robust Implementation**: Path validation, repair mechanisms, and error handling

Perfect for learning advanced algorithms, optimization techniques, and their application to real manufacturing problems!  
