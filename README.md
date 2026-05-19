# AutoDelivery--Vityarthi

Autonomous delivery path-planning simulation on a 2D grid using:
- **BFS**
- **UCS (Dijkstra)**
- **A\***
- **Local-search replanning** for dynamic obstacles

## What the project does

The agent moves from **S (start)** to **G (goal)** on map files with:
- `#` → blocked cell
- `.` / `0` → normal terrain (cost `1`)
- `1-9` → movement cost
- `M` → moving obstacle seed (used to build a deterministic obstacle schedule)

It supports:
- single-run planning
- dynamic demo with replanning
- experiment mode that saves metrics (`time`, `nodes_expanded`, `path_cost`) to CSV

## Repository structure

```text
.
├── autodelivery.py
├── small.map
├── medium.map
├── large.map
├── dynamic.map
├── results/
│   └── experiment_results.csv   (generated after experiments)
└── README.md
```

## Requirements

- Python **3.8+**
- No required third-party packages for core execution

## How to run

Run commands from the project root.

### 1) Run one planner on one map

```bash
python autodelivery.py --map small.map --planner astar
```

Available planners:
- `bfs`
- `ucs`
- `astar`

Example:
```bash
python autodelivery.py --map medium.map --planner ucs
```

### 2) Run dynamic demo (replanning)

```bash
python autodelivery.py --map dynamic.map --planner astar --dynamic-demo
```

Optional unpredictable obstacle behavior:
```bash
python autodelivery.py --map dynamic.map --planner astar --dynamic-demo --unpredictable
```

### 3) Run experiments on all maps

`--run-experiments` expects:
- map files inside `maps/`
- `results/` to be a directory

One-time setup:

```bash
mkdir -p maps
cp *.map maps/
rm -f results
mkdir -p results
```

Then run:

```bash
python autodelivery.py --run-experiments
```

This runs BFS/UCS/A* on:
- `small.map`
- `medium.map`
- `large.map`
- `dynamic.map`

Output CSV:
```text
results/experiment_results.csv
```

## CLI options

```text
--map <file>             map file path (default: small.map)
--planner <bfs|ucs|astar>
--dynamic-demo
--run-experiments
--unpredictable
```

## Notes

- Movement is 4-directional (up/down/left/right).
- A* heuristic uses Manhattan distance × minimum terrain cost.
- The moving obstacle schedule is currently generated programmatically from `M` markers.
