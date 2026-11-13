# Random Occupancy Grid Autopilot
An autonomous maze-solving system featuring probabilistic occupancy grids, stochastic terrain generation, and real-time pathfinding using D* algorithm with probabilistic noise in ROS with Rviz and Gazebo for simulating the real world physics.

<p align="center">
    <img width="1110" height="756" alt="image" src="https://github.com/user-attachments/assets/3b712f5c-e937-4b2b-aab8-131d150bbd6f" />
</p>

## Adaptive Path Clearance and Wall Avoidance

The pathfinding engine validates the neighboring quadrants around each candidate cell through the `has_clearance` function. This validation checks whether an obstacle is present within the configured clearance distance, allowing the A* algorithm to redirect the route before it reaches a wall or another blocked object.

The navigation cost includes a preference for paths that preserve one free cell between the route and nearby obstacles. When a candidate cell satisfies `clearance=1`, it keeps the preferred safety margin. If no route with this separation is available, the algorithm falls back to `clearance=0`, allowing it to pass through a narrow opening while still rejecting cells that are actual obstacles.

A penalty is added to candidates without the preferred clearance. This penalty does not make narrow passages impossible; it only makes them less attractive than equivalent paths that maintain one cell of separation. As a result, the planner avoids walls whenever possible while preserving the ability to find a valid path through constrained areas.

## Spatial Generation and Computational Cost Reduction

The `Map` class implements the representation and generation of the navigation environment through a stochastic occupancy grid. Instead of determining the state of each cell using a sequential random process, the system uses spatial hashing to calculate each cell's value independently based on its position and a seed.

This procedure generates values that appear random while remaining reproducible and consistent for the same position and seed. The primary advantage is that a cell's state can be obtained on demand without storing the entire map or iterating through all previous cells, reducing memory usage and computational overhead.
<p align="center">
    <img width="248" height="219" alt="image" src="https://github.com/user-attachments/assets/a5658819-cf5e-42a9-bb12-2c7fefc8dbbf" />
</p>

The `get_cell_cost` function adds an important exception to the probabilistic model: positions belonging to `carved_corridors` always receive a cost of `1.0`, even if the hash would otherwise classify them as obstacles or high-cost terrain. This guarantees that the generated maze contains traversable routes for the navigation algorithms.

From a scientific perspective, the design separates two processes with different purposes: *spatial hashing* deterministically controls the distribution of obstacles, while the path-carving process guarantees connectivity and solvability. This combination provides both stochastic variability and reproducible experiments.
<p align="center">
    <img width="1030" height="959" alt="image" src="https://github.com/user-attachments/assets/fc97a803-733a-498f-b10b-a83800ac1200" />
</p>

```mermaid
flowchart TD
    %% Node styles
    classDef startEnd fill:#F1C40F,stroke:#D68910,stroke-width:2px,color:#000;
    classDef process fill:#3498DB,stroke:#2980B9,stroke-width:2px,color:#fff;
    classDef condition fill:#E67E22,stroke:#D35400,stroke-width:2px,color:#fff;
    classDef sub fill:#9B59B6,stroke:#8E44AD,stroke-width:2px,color:#fff;

    %% Main flow
    START([Game start]):::startEnd
    M1_INIT["Module 1: Initialize an N × M grid"]:::process

    START --> M1_INIT
    M1_INIT --> M1_NOISE["Calculate continuous Perlin / Gaussian noise"]:::process
    M1_NOISE --> M1_CELL{"Is noise > threshold?"}:::condition

    M1_CELL -->|Yes| M1_WALL["Assign impassable obstacle<br/>Cost = ∞"]:::process
    M1_CELL -->|No| M1_COST["Assign continuous terrain<br/>Cost ≥ 1"]:::process

    M1_WALL --> M1_WAVE["Run the Wavefront validator from the goal"]:::sub
    M1_COST --> M1_WAVE

    M1_WAVE --> M1_VALID{"Is there an open and solvable path?"}:::condition
    M1_VALID -->|False| M1_INIT
    M1_VALID -->|True| M3_MODE["Module 3: Select game mode"]:::process

    %% Game modes
    M3_MODE --> M3_CHOICE{"Which mode?"}:::condition

    M3_CHOICE -->|MvM| M2_MVM["Autonomous competition:<br/>A* with Manhattan vs. D* / Wavefront"]:::process
    M3_CHOICE -->|HvM| M2_HVM["Human vs. CPU in real time"]:::process
    M3_CHOICE -->|HvH| M2_HVH["Local manual multiplayer"]:::process

    %% Navigation engine
    M2_MVM --> M2_ASTAR
    M2_HVM --> M2_ASTAR
    M2_ASTAR["Module 2: A* engine<br/>Evaluates f(n) = g(n) + h(n)"]:::sub

    M2_ASTAR --> M2_LOOP{"Is the OPEN list empty?"}:::condition
    M2_LOOP -->|Yes| M4_ERR["Module 4: Report route error"]:::process
    M2_LOOP -->|No| M2_POP["Extract the node with the lowest f(n)"]:::process

    M2_POP --> M2_META{"Is the current node the goal?"}:::condition
    M2_META -->|Yes| M4_METRICS["Module 4: Trace the path and display metrics<br/>(ms, nodes)"]:::process
    M2_META -->|No| M2_EXPAND["Expand orthogonal neighbors<br/>and add traversal costs c"]:::process

    M2_EXPAND --> M2_DYN{"Does a stochastic event occur<br/>or does the map change?"}:::condition
    M2_DYN -->|Yes| M2_DSTAR["Reverse replanning with D*"]:::sub
    M2_DYN -->|No| M2_LOOP

    M2_DSTAR --> M2_LOOP

    %% Completion
    M4_ERR --> END([End of game]):::startEnd
    M2_HVH --> M4_METRICS
    M4_METRICS --> END
```

To generate the pseudorandom maze dynamically, the `generate_map` method uses Python's `time` library to create a seed from `time.thread_time_ns()`. This value is assigned to `self.seed`, allowing each maze generation to use a different time-based seed while preserving deterministic spatial hashing and corridor generation for that specific maze.
