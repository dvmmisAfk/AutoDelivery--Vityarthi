# Autonomous Delivery Agent – CSA2001 Project 1

This project implements an **autonomous delivery agent** navigating a 2D grid city with static obstacles, terrain costs, and dynamic moving obstacles.  
It demonstrates **uninformed search (BFS, UCS)**, **informed search (A*)**, and a **local-search replanner** for dynamic environments.

---

## Features
- Grid world environment:
  - `#` = impassable obstacle  
  - `.` = normal terrain (cost 1)  
  - `1-9` = explicit terrain movement cost  
  - `S` = start  
  - `G` = goal  
  - `M` = moving obstacle (with deterministic schedule)  
- **Search algorithms**:
  - BFS (Breadth-First Search)
  - UCS (Uniform-Cost Search / Dijkstra)
  - A* Search with admissible heuristic (`h = Manhattan × min_cost`)
- **Dynamic replanning**:
  - Moving obstacles with deterministic schedules
  - Unpredictable obstacles (stochastic presence)
  - Local search–based replanner (simulated annealing style)
- **Metrics** collected:
  - Path cost
  - Nodes expanded
  - Runtime (seconds)
- CLI interface for running single planners, dynamic demos, or full experiments
- CSV experiment output for easy plotting/analysis

---

## Project Structure
```
autodelivery/
  maps/
    small.map
    medium.map
    large.map
    dynamic.map
  results/
    experiment_results.csv   # generated after experiments
  autodelivery.py            # main Python script
  README.md                  # this file
```

---

## Requirements
- Python **3.8+**
- No external dependencies for core algorithms
- Optional: `matplotlib`, `pandas` (for analysis/plots)

Install optional libraries:
```bash
pip install matplotlib pandas
```

---

## Usage

### 1. Run a planner on a map
```bash
python autodelivery.py --map maps/small.map --planner astar
```
Example output:
```
Loaded map maps/small.map size 5x5 start=Node(r=0,c=0) goal=Node(r=0,c=4)
Found path cost=7, nodes_expanded=14, time=0.0003s
Path: (0,0) -> (0,1) -> (0,2) -> (0,3) -> (0,4)
```

### 2. Dynamic demo with replanning
```bash
python autodelivery.py --map maps/dynamic.map --planner astar --dynamic-demo
```
Logs will show:
```
[t=0] Move to (0,0)
[t=1] Move to (0,1)
[t=2] Obstacle at (1,2) blocks path. Replanning...
[t=2] Replanning success. New path length 9.
...
[t=9] Goal reached.
```

### 3. Run full experiments (saves results to CSV)
```bash
python autodelivery.py --run-experiments
```
This will run BFS, UCS, and A* on `small.map`, `medium.map`, `large.map`, and `dynamic.map`.  
Results are saved to:
```
results/experiment_results.csv
```

---

## Maps

### Example `small.map`
```
S . . . G
. # # . .
. . 3 . .
. # . . .
. . . . .
```

### Example `dynamic.map`
```
S . . . . . . G
. . M . . . . .
. . . . . . . .
. . . . . . . .
. . . . . . . .
```
- `M` moves rightward then leftward in a repeating cycle.

---

## Report Guidance
Include in your short report:
1. **Environment model**: grid, costs, obstacles, moving obstacle schedules.  
2. **Agent design**: BFS, UCS, A*, heuristic, local replanner.  
3. **Experimental results**:  
   - Use `results/experiment_results.csv` to make tables/plots.  
   - Compare path cost, nodes expanded, runtime.  
4. **Dynamic demo proof**: screenshots or logs from `--dynamic-demo`.  
5. **Analysis**: trade-offs between BFS, UCS, A*, and replanning.  

---

## Future Improvements
- Add 8-connected movement (diagonal steps).  
- Richer map format (`@SCHEDULE` for explicit moving obstacle paths).  
- Graphical visualization (matplotlib animation).  
- Modularize into packages for cleaner structure.  

---

## License
This project is for **academic use** (CSA2001 course).  
Feel free to adapt/extend for your own experiments.
