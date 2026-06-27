# Mapping and Navigation in the Queen Elizabeth Olympic Park

A MATLAB-based mapping, path-planning, human–robot interaction, and sensor-data analysis project developed around the Queen Elizabeth Olympic Park in London.

The project models the park as both an occupancy grid and a weighted topological graph. It implements several navigation-related data structures and algorithms, provides an interactive graphical user interface for route planning, and analyses pedestrian motion data collected using MATLAB Mobile.

## Project Overview

The project is divided into two main parts.

### Part I: Mapping and Navigation

Part I focuses on:

* converting GPS coordinates into local Cartesian coordinates;
* generating an occupancy grid of the Olympic Park;
* representing navigable routes as a weighted graph;
* implementing linked-list, KD-tree, BFS, and Dijkstra-based methods;
* comparing path-planning algorithms;
* providing an interactive GUI for shortest-path queries.

### Part II: Sensor Data Analysis

Part II analyses mobile-phone sensor data by:

* converting GPS coordinates into a local coordinate system;
* estimating velocity using finite differences;
* applying moving-average filtering;
* estimating velocity from accelerometer integration;
* comparing multiple speed estimates;
* classifying motion as stopped, slow walking, fast movement, or turning.

## Repository Structure

```text
.
├── README.md
├── Report.pdf
├── demo
├── sensorlog.mat
└── code/
    ├── PartI-Task1-Grid.m
    ├── PartI-Task2-Algorithms.m
    ├── PartI-Task3-GUI.m
    └── PartII.m
```

| File                            | Description                                                          |
| ------------------------------- | -------------------------------------------------------------------- |
| `README.md`                     | Project overview and usage instructions                              |
| `Report.pdf`                    | Full coursework report, methodology, results, and discussion         |
| `demo`                          | Demonstration media for the project                                  |
| `sensorlog.mat`                 | MATLAB Mobile position and acceleration dataset                      |
| `code/PartI-Task1-Grid.m`       | Occupancy-grid construction and map visualisation                    |
| `code/PartI-Task2-Algorithms.m` | Graph construction, search algorithms, KD-tree, and navigation logic |
| `code/PartI-Task3-GUI.m`        | Interactive graphical path-planning interface                        |
| `code/PartII.m`                 | GPS, acceleration, filtering, and motion-pattern analysis            |

## Part I — Mapping and Navigation

## Task 1: Occupancy Grid Construction

`PartI-Task1-Grid.m` constructs a local map of the Queen Elizabeth Olympic Park from a set of GPS landmarks.

The script contains:

* 7 key locations labelled `K1` to `K7`;
* 10 signal or intermediate locations labelled `S1` to `S10`;
* circular and polygonal obstacle regions;
* buildings, rivers, and inaccessible map regions;
* a resolution of approximately 1 metre per grid cell.

GPS latitude and longitude values are converted into local Cartesian coordinates using an Earth-radius approximation:

```matlab
R = 6371000;

x = R * cosd(lat0) * deg2rad(lon - lon0);
y = R * deg2rad(lat - lat0);
```

The local coordinates are shifted into the positive occupancy-map coordinate space before obstacles and navigation points are added.

### Map Representation

The map uses:

* filled markers for key points;
* hollow markers for signal points;
* occupied cells for buildings, water, and restricted areas;
* free cells for potentially navigable regions.

The occupancy map provides a spatial representation of the park while the graph used in Task 2 provides a simplified topological representation for route planning.

## Task 2: Navigation Algorithms

`PartI-Task2-Algorithms.m` implements the graph and data structures used for route planning.

### Weighted Graph

The locations are represented as a sparse weighted graph.

* Nodes represent key and signal points.
* Edges represent walkable connections.
* Edge weights represent geographical distance.

The graph is stored using an adjacency list:

```matlab
adj = cell(N, 1);
```

Distances between GPS coordinates are calculated using the Haversine formula.

### Implemented Algorithms

#### Breadth-First Search

Breadth-first search finds a path with the minimum number of graph edges.

```text
Time complexity: O(V + E)
```

BFS is useful when all transitions are treated equally, but it does not guarantee the shortest physical distance when edge weights differ.

#### Array-Based Dijkstra

The array-based Dijkstra implementation repeatedly scans all unvisited nodes to find the node with the smallest tentative distance.

```text
Time complexity: O(V²)
```

It guarantees a shortest-distance path but is less efficient for larger graphs or repeated route queries.

#### Heap-Based Dijkstra

The heap-based implementation uses a manually implemented binary min-heap as a priority queue.

```text
Time complexity: O((V + E) log V)
```

This version is more suitable for repeated shortest-path calculations and larger navigation graphs.

The heap operations include:

```matlab
heapPush(...)
heapPop(...)
```

#### KD-Tree

A two-dimensional KD-tree is implemented for nearest-landmark queries.

The tree recursively alternates between the X and Y axes and supports nearest-neighbour lookup among key landmarks.

```text
Average nearest-neighbour query: O(log n)
Linear search: O(n)
```

The KD-tree is useful in tourist-guide scenarios where the robot must identify the closest unvisited landmark.

#### Linked List

A linked-list representation is also explored for flexible storage of navigation locations.

Linked lists allow efficient insertion and deletion, but nearest-point lookup requires linear traversal.

```text
Search complexity: O(n)
```

## Navigation Scenarios

The algorithms support several representative navigation scenarios.

### Mandatory Intermediate Point

A route can be divided into multiple shortest-path queries when the robot must visit a required intermediate location.

```text
Start → Required waypoint → Destination
```

Heap-based Dijkstra is appropriate because it preserves shortest-distance guarantees for each route segment.

### Tourist Guide Mode

A tourist-guide mode can combine:

1. a KD-tree to select the nearest unvisited landmark;
2. heap-based Dijkstra to calculate the shortest route to that landmark.

### Repeated Route Queries

For repeated navigation between different destinations, the heap-based Dijkstra implementation avoids the repeated full-array scans required by array-based Dijkstra.

## Task 3: Interactive Navigation GUI

`PartI-Task3-GUI.m` provides an interactive MATLAB interface called:

```text
QEOP Navigation System
```

The interface includes:

* a start-node dropdown;
* a goal-node dropdown;
* a `Find Shortest Path` button;
* an occupancy-map visualisation;
* highlighted key and signal points;
* a plotted shortest route;
* calculated route distance;
* path-planning execution time;
* a status display.

### GUI Workflow

```text
Select start node
        │
        ▼
Select destination
        │
        ▼
Press "Find Shortest Path"
        │
        ▼
Run heap-based Dijkstra
        │
        ▼
Plot route on occupancy map
        │
        ▼
Display distance and execution time
```

The GUI uses event-driven callbacks so that route calculations occur when the user requests them.

Example interface creation:

```matlab
fig = uifigure(...);
ax = uiaxes(fig, ...);
ddStart = uidropdown(...);
ddGoal = uidropdown(...);
btnNav = uibutton(...);
```

The selected route is plotted over the occupancy map, and the result is displayed in the status area.

## Part II — Mobile Sensor Data Analysis

`PartII.m` processes position and acceleration data stored in `sensorlog.mat`.

## Data Sources

The dataset contains:

### Position Data

* latitude;
* longitude;
* altitude;
* phone-reported speed;
* heading or course;
* timestamps.

### Acceleration Data

* X-axis acceleration;
* Y-axis acceleration;
* Z-axis acceleration;
* timestamps.

## GPS Coordinate Conversion

The first GPS sample is used as the origin.

Latitude, longitude, and altitude are converted into local coordinates:

```matlab
x = R * cosd(lat0) .* deg2rad(lon - lon0);
y = R * deg2rad(lat - lat0);
z = alt - alt(1);
```

This produces a local trajectory in metres.

## Velocity Estimation

Velocity components are calculated using numerical differentiation:

```matlab
vx = diff(x) ./ dt;
vy = diff(y) ./ dt;
vz = diff(z) ./ dt;
```

The total GPS-derived speed is then calculated as:

```matlab
v_gps_raw = sqrt(vx.^2 + vy.^2 + vz.^2);
```

## Moving-Average Filtering

A moving-average filter is applied independently to the velocity components:

```matlab
win = 3;

vx_f = movmean(vx, win);
vy_f = movmean(vy, win);
vz_f = movmean(vz, win);
```

The filtered total speed is:

```matlab
v_gps_f = sqrt(vx_f.^2 + vy_f.^2 + vz_f.^2);
```

Filtering reduces short-term GPS fluctuations while retaining the overall movement trend.

## Accelerometer-Based Velocity

The acceleration magnitude is calculated using:

```matlab
a_mag = sqrt(ax.^2 + ay.^2 + az.^2);
```

A moving estimate of gravity is removed:

```matlab
g_est = movmean(a_mag, 20);
a_dyn = a_mag - g_est;
```

Small residual values are suppressed:

```matlab
a_dyn(abs(a_dyn) < 0.1) = 0;
```

Velocity is then estimated by cumulative trapezoidal integration:

```matlab
v_acc_corr = cumtrapz(t_acc_sec, a_dyn);
```

The accelerometer-derived estimate is compared with:

* filtered GPS-derived speed;
* phone-reported GPS speed.

## Motion Pattern Detection

The script classifies motion using filtered speed thresholds.

```matlab
if v_gps_f(i) < 0.5
    pattern(i) = "Stop";
elseif v_gps_f(i) < 1.2
    pattern(i) = "Slow walk";
else
    pattern(i) = "Fast";
end
```

Turning is detected using changes in GPS course:

```matlab
if dcourse(i) > 20 && v_gps_f(i) > 0.5
    pattern(i) = "Turning";
end
```

The detected categories are:

* `Stop`
* `Slow walk`
* `Fast`
* `Turning`

The classified states are plotted along the recorded trajectory.

## Generated Outputs

Running `PartII.m` generates:

1. raw and filtered X-direction velocity;
2. raw and filtered Y-direction velocity;
3. raw and filtered Z-direction velocity;
4. GPS and accelerometer speed comparison;
5. motion-pattern labels on the walking trajectory;
6. a results table containing filtered velocity and pattern classifications;
7. summary counts for each detected motion type;
8. RMSE between filtered GPS-derived speed and phone GPS speed.

## Requirements

The project requires MATLAB with support for:

* `occupancyMap`;
* `uifigure`;
* `uiaxes`;
* `uidropdown`;
* `uibutton`;
* MATLAB table and plotting functions.

The exact toolbox requirements may vary between MATLAB versions.

A recent MATLAB release is recommended.

## Running the Project

Clone the repository:

```bash
git clone https://github.com/YOUR_USERNAME/Mapping-and-Navigation-in-the-Olympic-Park.git
cd Mapping-and-Navigation-in-the-Olympic-Park
```

Open MATLAB and set the repository as the current folder.

## Run Task 1

```matlab
run("code/PartI-Task1-Grid.m")
```

This creates and displays the occupancy grid.

## Run Task 2

Task 2 depends on the GPS and local-coordinate variables created in Task 1.

Run the files in this order:

```matlab
run("code/PartI-Task1-Grid.m")
run("code/PartI-Task2-Algorithms.m")
```

The script constructs the graph and runs the implemented data-structure and path-planning logic.

## Run Task 3

The GUI file defines the navigation interface.

Because the main function in the submitted file is named `QEOP_Navigation_System`, MATLAB may require the file to be renamed to:

```text
QEOP_Navigation_System.m
```

After renaming, run:

```matlab
QEOP_Navigation_System
```

Alternatively, rename the function inside the file so that its name matches `PartI-Task3-GUI.m`.

MATLAB requires the primary function name and filename to match.

## Run Part II

Before running `PartII.m`, update the dataset path.

The current source contains a machine-specific absolute path:

```matlab
load("C:\Users\a1805\Desktop\sensorlog.mat");
```

For repository use, replace it with:

```matlab
load("sensorlog.mat");
```

Then run:

```matlab
run("code/PartII.m")
```

## Algorithm Summary

| Method                  | Purpose                          |  Typical complexity |
| ----------------------- | -------------------------------- | ------------------: |
| Linked list             | Flexible landmark storage        |      Search: `O(n)` |
| KD-tree                 | Nearest-landmark lookup          | Average: `O(log n)` |
| Adjacency list          | Sparse graph storage             |    `O(V + E)` space |
| BFS                     | Minimum-edge path                |          `O(V + E)` |
| Array Dijkstra          | Shortest weighted path           |             `O(V²)` |
| Heap Dijkstra           | Efficient shortest weighted path |  `O((V + E) log V)` |
| Moving average          | GPS velocity smoothing           |              `O(n)` |
| Finite difference       | Velocity estimation              |              `O(n)` |
| Trapezoidal integration | Acceleration-based velocity      |              `O(n)` |

## Key Design Decisions

### Occupancy Grid and Topological Graph

The occupancy grid represents physical obstacles and free space, while the graph captures meaningful navigation routes between landmarks.

Using both representations separates:

* low-level spatial mapping;
* high-level route planning.

### Static Map Coordinates

Static obstacles are manually represented using known geographical coordinates rather than relying only on noisy recorded GPS trajectories.

This reduces map distortion near large structures and water features.

### One-Metre Grid Resolution

A resolution of approximately one metre per cell balances:

* environmental detail;
* memory use;
* computational cost.

### Heap-Based Route Planning

Heap-based Dijkstra is used in the GUI because it guarantees a shortest weighted path while remaining suitable for repeated interactive queries.

### Event-Driven GUI

The GUI calculates routes only after a user selects the start and destination and presses the route-planning button.

This avoids unnecessary precomputation and provides immediate visual feedback.

## Known Limitations

* The map obstacles are manually approximated using circles and polygons.
* GPS-to-Cartesian conversion uses a local Earth approximation.
* The graph contains only predefined navigable connections.
* BFS optimises edge count rather than geographical distance.
* The KD-tree is used with a small landmark dataset.
* GPS course can be noisy at low walking speeds.
* Accelerometer integration accumulates drift over time.
* Motion classification uses manually selected thresholds.
* The GUI source filename does not currently match its primary MATLAB function.
* `PartII.m` contains a machine-specific absolute dataset path.
* Some scripts depend on variables created by earlier scripts rather than being fully self-contained.

## Possible Improvements

* Store all GPS coordinates and graph data in a shared configuration file.
* Convert each task into reusable MATLAB functions.
* Make Task 2 independent of variables created by Task 1.
* Add A* search for occupancy-grid path planning.
* Automatically generate graph edges from walkable map regions.
* Use geospatial mapping tools for more accurate projections.
* Add path caching for repeated route queries.
* Add algorithm runtime comparisons and benchmark plots.
* Use a MATLAB priority-queue abstraction if available.
* Improve KD-tree visualisation.
* Add automatic GPS outlier rejection.
* Apply low-pass or Kalman filtering to sensor data.
* Perform orientation compensation before integrating acceleration.
* Replace fixed motion thresholds with learned classification models.
* Export route and sensor-analysis results automatically.

## Report

The complete methodology, justification, algorithm comparison, GUI discussion, results, and numerical analysis are available in:

```text
Report.pdf
```

## Academic Context

This repository contains coursework involving:

* mapping and occupancy grids;
* graph data structures;
* path-planning algorithms;
* human–robot interaction;
* numerical methods;
* mobile sensor-data processing.

The repository is published for portfolio and demonstration purposes. Current students should follow their institution's academic-integrity requirements before reusing any part of the implementation.
