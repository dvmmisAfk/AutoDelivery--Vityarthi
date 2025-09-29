"""
autodelivery.py
Single-file implementation for CSA2001 Project 1 proof-of-concept.

Features:
 - Grid maps with integer movement costs
 - Static obstacles (#), variable terrains (digits), S start, G goal
 - Deterministic moving obstacles (schedule) or unpredictable mode
 - Planners: BFS, UCS (Dijkstra), A* with admissible heuristic
 - Local-search based replanner (simulated annealing style) for dynamic replanning
 - CLI for running planners, running dynamic demo, and collecting metrics

Usage examples:
  python autodelivery.py --map maps/small.map --planner astar
  python autodelivery.py --map maps/dynamic.map --planner astar --dynamic-demo
  python autodelivery.py --run-experiments
"""

import argparse
import time
import heapq
import math
import random
import csv
from copy import deepcopy
from collections import deque, defaultdict, namedtuple

# -------------------------
# Utilities & types
# -------------------------
Node = namedtuple("Node", ["r","c"])
DIRS = [(-1,0),(1,0),(0,-1),(0,1)]  # 4-connected
INF = 10**9

def neighbors(r,c,rows,cols,allow_diag=False):
    for dr,dc in DIRS:
        nr, nc = r+dr, c+dc
        if 0 <= nr < rows and 0 <= nc < cols:
            yield nr,nc

def manhattan(a,b):
    return abs(a.r-b.r)+abs(a.c-b.c)

# -------------------------
# Grid / Environment
# -------------------------
class Grid:
    def __init__(self, grid, start, goal, moving_schedules=None):
        """
        grid: 2D list of ints or None for obstacles
        start, goal: Node
        moving_schedules: dict[id] -> list of (t, Node) deterministic schedule
        """
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0]) if self.rows>0 else 0
        self.start = start
        self.goal = goal
        self.moving = moving_schedules or {}  # map id -> list of (t,Node) sorted by t

    @classmethod
    def from_lines(cls, lines):
        # parse simple map format; ignore comment lines starting with #
        grid = []
        start = goal = None
        moving_positions = []
        for r,line in enumerate(lines):
            line=line.strip()
            if line=="" or line.startswith("#"): 
                continue
            tokens = line.split()
            row = []
            for c,tok in enumerate(tokens):
                if tok == '#':
                    row.append(None)
                elif tok == '.' or tok=='0':
                    row.append(1)
                elif tok == 'S':
                    row.append(1)
                    start = Node(r, len(row)-1)
                elif tok == 'G':
                    row.append(1)
                    goal = Node(r, len(row)-1)
                elif tok == 'M':
                    # treat as free cell but note moving obstacle initial pos
                    row.append(1)
                    moving_positions.append(Node(r, len(row)-1))
                elif tok.isdigit():
                    row.append(int(tok))
                else:
                    # fallback plain terrain
                    row.append(1)
            grid.append(row)
        # For this simple parser, if map uses 'S' and 'G' in different columns
        # we found start/goal but the row indices in start/goal above may not match due to comments filtered.
        # To make robust, re-scan grid to locate S/G if still None
        if start is None or goal is None:
            for r,row in enumerate(grid):
                for c,val in enumerate(row):
                    # can't detect S/G if removed earlier; assume first cell is S, last is G (fallback)
                    pass
        # dynamic schedule example: if a single moving pos exists, create deterministic rightward movement
        schedules = {}
        if moving_positions:
            # create a simple deterministic schedule for first moving object
            m = moving_positions[0]
            # repeat cycle moving right 4 steps then left
            sched=[]
            width = len(grid[0])
            for t in range(0, 20):
                # simple back and forth
                offset = (t % 8)
                if offset <= 3:
                    nc = m.c + offset
                else:
                    nc = m.c + (6 - offset)
                if nc >= width: nc = width-1
                sched.append((t, Node(m.r, nc)))
            schedules["veh0"] = sched
        return cls(grid, start, goal, schedules)

    def in_bounds(self, r,c):
        return 0 <= r < self.rows and 0 <= c < self.cols

    def passable(self, r,c, t=None, unpredictable_mode=False):
        """Return True if (r,c) not statically blocked and not occupied by a moving obstacle at time t (if given)."""
        if not self.in_bounds(r,c): return False
        if self.grid[r][c] is None: return False
        if t is not None:
            for mid, sched in self.moving.items():
                # find position at time t (schedule gives discrete times; pick nearest <= t)
                pos = self._pos_at_time(sched, t)
                if pos and pos.r==r and pos.c==c:
                    # In unpredictable mode, sometimes moving obstacle not present
                    if unpredictable_mode:
                        # 70% chance it is present (for demo)
                        if random.random() < 0.7:
                            return False
                        else:
                            continue
                    return False
        return True

    def cost(self, r,c):
        return self.grid[r][c]

    def _pos_at_time(self, sched, t):
        # sched: list of (t0, Node)
        if not sched: return None
        # schedule assumed to have entry for each t or repeating pattern:
        # find last entry with time <= t mod cycle
        # we'll implement as looping using len(sched)
        idx = t % len(sched)
        return sched[idx][1]

# -------------------------
# Search algorithms
# -------------------------
def bfs_search(grid: Grid, allow_diag=False, dynamic_time=False, unpredictable_mode=False):
    start = grid.start
    goal = grid.goal
    rows,cols = grid.rows, grid.cols
    frontier = deque()
    frontier.append((start,0))  # Node, time (steps)
    came_from = { (start.r,start.c): None }
    expanded = 0
    while frontier:
        node, t = frontier.popleft()
        expanded += 1
        if node == goal:
            # reconstruct path
            path=[]
            cur = (node.r,node.c)
            while cur:
                path.append(Node(cur[0],cur[1]))
                cur = came_from[cur]
            path.reverse()
            return {"path":path, "cost": sum(grid.cost(n.r,n.c) for n in path), "expanded":expanded, "time":t}
        for nr,nc in neighbors(node.r,node.c,rows,cols,allow_diag):
            if not grid.passable(nr,nc, t+1, unpredictable_mode): continue
            if (nr,nc) not in came_from:
                came_from[(nr,nc)] = (node.r,node.c)
                frontier.append((Node(nr,nc), t+1))
    return None

def ucs_search(grid: Grid, allow_diag=False, unpredictable_mode=False):
    start = grid.start
    goal = grid.goal
    rows,cols = grid.rows, grid.cols
    pq = []
    heapq.heappush(pq, (0, start, 0))  # (g, node, t)
    came_from = {}
    best_cost = { (start.r,start.c): 0 }
    expanded = 0
    while pq:
        g, node, t = heapq.heappop(pq)
        expanded += 1
        if (node.r,node.c) == (goal.r,goal.c):
            # reconstruct path
            path=[]
            cur = (node.r,node.c)
            while cur != (start.r,start.c):
                path.append(Node(cur[0],cur[1]))
                cur = came_from[cur]
            path.append(start)
            path.reverse()
            return {"path":path, "cost":g, "expanded":expanded, "time":t}
        for nr,nc in neighbors(node.r,node.c,rows,cols,allow_diag):
            nt = t+1
            if not grid.passable(nr,nc, nt, unpredictable_mode): continue
            step_cost = grid.cost(nr,nc)
            newg = g + step_cost
            if (nr,nc) not in best_cost or newg < best_cost[(nr,nc)]:
                best_cost[(nr,nc)] = newg
                came_from[(nr,nc)] = (node.r,node.c)
                heapq.heappush(pq, (newg, Node(nr,nc), nt))
    return None

def astar_search(grid: Grid, allow_diag=False, unpredictable_mode=False):
    start = grid.start
    goal = grid.goal
    rows,cols = grid.rows, grid.cols
    # admissible heuristic: manhattan * min_cost_in_grid (min terrain cost >=1)
    min_cost = min([grid.grid[r][c] for r in range(rows) for c in range(cols) if grid.grid[r][c] is not None] or [1])
    def h(node):
        return manhattan(node, goal) * min_cost
    pq = []
    gscore = { (start.r,start.c): 0 }
    fscore = { (start.r,start.c): h(start) }
    heapq.heappush(pq, (fscore[(start.r,start.c)], 0, start, 0))  # (f, g, node, t)
    came_from = {}
    expanded = 0
    while pq:
        f, g, node, t = heapq.heappop(pq)
        expanded += 1
        if (node.r,node.c) == (goal.r,goal.c):
            # reconstruct path
            path=[]
            cur = (node.r,node.c)
            while cur != (start.r,start.c):
                path.append(Node(cur[0],cur[1]))
                cur = came_from[cur]
            path.append(start)
            path.reverse()
            return {"path":path, "cost":g, "expanded":expanded, "time":t}
        for nr,nc in neighbors(node.r,node.c,rows,cols,allow_diag):
            nt = t+1
            if not grid.passable(nr,nc, nt, unpredictable_mode): continue
            tentative_g = g + grid.cost(nr,nc)
            if (nr,nc) not in gscore or tentative_g < gscore[(nr,nc)]:
                gscore[(nr,nc)] = tentative_g
                fnew = tentative_g + h(Node(nr,nc))
                came_from[(nr,nc)] = (node.r,node.c)
                heapq.heappush(pq, (fnew, tentative_g, Node(nr,nc), nt))
    return None

# -------------------------
# Local search replanner (simulated annealing style)
# -------------------------
def local_search_replan(grid: Grid, current_path, current_time, unpredictable_mode=False, time_budget=0.02):
    """
    Simple local-search that attempts to repair the remaining path when obstacle appears.
    current_path: list of Node from current position to goal
    current_time: current timestep
    Returns new_path or None.
    """
    # We'll implement a simple randomized-walk + hill-climb:
    start = current_path[0]
    goal = current_path[-1]
    best = current_path
    best_cost = sum(grid.cost(n.r,n.c) for n in best)
    start_time = time.time()
    iter_count = 0
    T0 = 1.0
    while time.time() - start_time < time_budget and iter_count < 500:
        iter_count += 1
        # propose a local modification: insert a random detour by selecting a random position near start
        # Build a short random walk using DFS for length L
        L = random.randint(1,4)
        cur = start
        walk = [cur]
        for _ in range(L):
            cand = []
            for nr,nc in neighbors(cur.r,cur.c,grid.rows,grid.cols):
                if grid.passable(nr,nc, current_time+len(walk), unpredictable_mode):
                    cand.append(Node(nr,nc))
            if not cand: break
            cur = random.choice(cand)
            walk.append(cur)
        # then try to connect last of 'walk' to goal via greedy A* (limited)
        tail_start = walk[-1]
        # limited A* from tail_start
        subgrid = deepcopy(grid)
        subgrid.start = tail_start
        subgrid.goal = goal
        res = astar_search(subgrid, unpredictable_mode=unpredictable_mode)
        if res is None: continue
        candidate = walk[:-1] + res['path']
        cand_cost = sum(grid.cost(n.r,n.c) for n in candidate)
        # acceptance: if better, accept. Else accept with small prob depending on T.
        T = T0 * (0.99 ** iter_count)
        if cand_cost < best_cost or random.random() < math.exp(-(cand_cost-best_cost)/max(T,1e-6)):
            best = candidate
            best_cost = cand_cost
    return best

# -------------------------
# Runner & dynamic demo
# -------------------------
def run_planner(grid: Grid, planner_name="astar", unpredictable_mode=False, time_limit=60.0):
    start_time = time.time()
    planner = {"bfs":bfs_search, "ucs":ucs_search, "astar":astar_search}[planner_name]
    res = planner(grid, unpredictable_mode=unpredictable_mode)
    end_time = time.time()
    if res is None:
        return {"success":False, "time":end_time-start_time, "expanded":None}
    return {"success":True, "time":end_time-start_time, "expanded":res["expanded"], "cost":res["cost"], "path":res["path"]}

def dynamic_demo(grid: Grid, planner_name="astar", unpredictable_mode=False, replanner="local", verbose=True):
    """
    Simulate agent executing planned path and show dynamic obstacles causing replanning.
    """
    timeline = []
    t = 0
    # initial plan
    plan = run_planner(grid, planner_name, unpredictable_mode)
    if not plan.get("success"):
        print("No initial plan found.")
        return
    path = plan["path"]
    cur_idx = 0
    logs=[]
    while cur_idx < len(path):
        cur = path[cur_idx]
        # check occupancy at next time for moving obstacles
        if not grid.passable(cur.r, cur.c, t, unpredictable_mode):
            # obstacle present on planned cell -> need to replan from previous safe position
            logs.append(f"[t={t}] Obstacle at {cur} blocks path. Replanning...")
            # current agent position is previous cell (or this if starting)
            agent_pos = path[max(0, cur_idx-1)]
            # prepare remaining path from agent_pos to goal
            remaining = [agent_pos] + path[cur_idx:]
            if replanner == "local":
                new_path = local_search_replan(grid, remaining, t, unpredictable_mode)
            else:
                # full replan
                grid.start = agent_pos
                rp = run_planner(grid, planner_name, unpredictable_mode)
                new_path = rp.get("path")
            if new_path is None:
                logs.append(f"[t={t}] Replanning failed.")
                return {"logs":logs}
            # reattach, set current index to 0 of new_path (agent_pos)
            path = new_path
            cur_idx = 0
            logs.append(f"[t={t}] Replanning success. New path length {len(path)}.")
        else:
            # step into the cell
            logs.append(f"[t={t}] Move to {cur}")
            cur_idx += 1
            t += 1
            # time advances; continue
    logs.append(f"[t={t}] Goal reached.")
    if verbose:
        for L in logs: print(L)
    return {"logs":logs, "steps":t}

# -------------------------
# Helpers: load map file
# -------------------------
def load_map_file(path):
    with open(path,'r') as f:
        lines = [ln.rstrip("\n") for ln in f.readlines()]
    g = Grid.from_lines(lines)
    # If Grid.from_lines didn't set start/goal properly, try to locate 'S' and 'G' in file tokens:
    # Re-parse to find positions robustly
    tokens=[]
    clean=[]
    r=0
    start=goal=None
    for line in lines:
        if line.strip()=="" or line.strip().startswith("#"):
            continue
        rowT = line.split()
        newrow=[]
        c=0
        for tok in rowT:
            if tok=='S':
                start = Node(r,c); newrow.append('.')
            elif tok=='G':
                goal = Node(r,c); newrow.append('.')
            else:
                newrow.append(tok)
            c+=1
        clean.append(" ".join(newrow))
        r+=1
    if start is not None: g.start = start
    if goal is not None: g.goal = goal
    return g

# -------------------------
# CLI and experiments
# -------------------------
def run_experiments(map_paths, planners=["bfs","ucs","astar"], repeats=3, unpredictable=False):
    results=[]
    for mp in map_paths:
        grid = load_map_file(mp)
        for p in planners:
            for r in range(repeats):
                random.seed(r)
                t0=time.time()
                out = run_planner(grid, p, unpredictable_mode=unpredictable)
                t1=time.time()
                results.append({
                    "map": mp,
                    "planner": p,
                    "run": r,
                    "success": out.get("success",False),
                    "time_s": out.get("time", t1-t0),
                    "nodes_expanded": out.get("expanded"),
                    "path_cost": out.get("cost"),
                })
    # save CSV
    fname = "results/experiment_results.csv"
    with open(fname,'w',newline='') as f:
        writer=csv.DictWriter(f, fieldnames=list(results[0].keys()))
        writer.writeheader()
        for row in results:
            writer.writerow(row)
    print(f"Saved experiment results to {fname}")
    return results

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", type=str, default="maps/small.map")
    parser.add_argument("--planner", choices=["bfs","ucs","astar"], default="astar")
    parser.add_argument("--dynamic-demo", action="store_true")
    parser.add_argument("--run-experiments", action="store_true")
    parser.add_argument("--unpredictable", action="store_true")
    args = parser.parse_args()

    if args.run_experiments:
        maps = ["maps/small.map","maps/medium.map","maps/large.map","maps/dynamic.map"]
        run_experiments(maps, planners=["bfs","ucs","astar"], repeats=3, unpredictable=args.unpredictable)
        return

    grid = load_map_file(args.map)
    print(f"Loaded map {args.map} size {grid.rows}x{grid.cols} start={grid.start} goal={grid.goal}")
    if args.dynamic_demo:
        print("Starting dynamic demo with replanning...")
        res = dynamic_demo(grid, planner_name=args.planner, unpredictable_mode=args.unpredictable, replanner="local")
        return
    # single-run
    out = run_planner(grid, args.planner, unpredictable_mode=args.unpredictable)
    if not out.get("success"):
        print("Planner failed to find a path.")
    else:
        print(f"Found path cost={out['cost']}, nodes_expanded={out['expanded']}, time={out['time']:.4f}s")
        print("Path:", " -> ".join(f"({n.r},{n.c})" for n in out['path']))

if __name__=="__main__":
    main()