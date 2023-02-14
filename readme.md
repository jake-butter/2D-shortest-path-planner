# Coding Assignment: 2D Shortest Path Planner
This program computes a solution to a given two-dimensional continuous-space shortest path problem.

The program takes as input start and goal points and a list of obstacles represented by simple polygons, and returns the shortest path from the start to the goal.

## Method Details
The program solves the shortest path problem by first computing a [visibility graph](https://en.wikipedia.org/wiki/Visibility_graph) for the problem, then finding the shortest path in the graph by Euclidean distance using [Dijkstra's algorithm](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm).

### Assumptions and Limitations
The path is assumed to represent the movement of a robot of infinitely small size, meaning that a path can move exactly along the edge of obstacles and the bounds of the problem environment.

All obstacles defined in the problem must be [simple polygons](https://en.wikipedia.org/wiki/Simple_polygon), meaning that they do not intersect and do not contain holes. Obstacles can intersect the environment boundaries and each other.

## Dependencies
- [NumPy](https://numpy.org/)
- [SciPy](https://scipy.org/)

## Usage
```
python planner.py config.yaml [output] [--plot]
```
- `config.yaml`: Must be specified and contains the problem definition in the format described below.
- `output`: Specifies the output file to write the path data to (default: `solution.txt`).
- `--plot`: When set, the program will output visualisations of the problem environment and the computed solution.

### Configuration
The program takes a configuration file in `yaml` format to define the navigation problem to be solved:

```
x_start: 2
y_start: 2
x_goal: 98
y_goal: 98
x_space_size: 100
y_space_size: 100
list_obstacles: [
  [[5,5], [8,12], [10,5]],
  [[50,60], [60,80], [80,90], [70,40]],
  [[20, 20], [10, 60], [40, 40], [40,30], [40, 20]],
  [[50, 20], [50, 45], [90, 30], [80, 5], [70, 10]]
]
```

The available options are as follows:

|Option|Explanation|
|---|---|
|`x_start`|x-coordinate of the robot start point|
|`y_start`|y-coordinate of the robot start point|
|`x_goal`|x-coordinate of the goal point|
|`y_goal`|y-coordinate of the goal point|
|`x_space_size`|The extent of the problem environment in the x-axis, from `0` to this value.|
|`y_space_size`|The extent of the problem environment in the y-axis, from `0` to this value.|
|`list_obstacles`|A list of obstacles for the robot to avoid, each obstacle being represented as a list of xy coordinates defining the vertices of a simple polygon. Obstacle vertices should be written in either clockwise or anti-clockwise order.|

## Examples
### Multiple obstacles
![Example 1](/examples/example1.png)
```
x_start: 2
y_start: 2
x_goal: 98
y_goal: 98
x_space_size: 100
y_space_size: 100
list_obstacles: [
  [[5,5], [8,12], [10,5]],
  [[50,60], [60,80], [80,90], [70,40]],
  [[20, 20], [10, 60], [40, 40], [40,30], [40, 20]],
  [[50, 20], [50, 45], [90, 30], [80, 5], [70, 10]]
]
```

### Obstacles exceed bounds
![Example 2](/examples/example2.png)
```
x_start: 2
y_start: 2
x_goal: 98
y_goal: 98
x_space_size: 100
y_space_size: 100
list_obstacles: [
  [[20, -10], [30, 80], [40, -10]],
  [[60, 110], [70, 20], [80, 110]]
]
```

### Concave obstacle
![Example 3](/examples/example3.png)
```
x_start: 50
y_start: 50
x_goal: 98
y_goal: 98
x_space_size: 100
y_space_size: 100
list_obstacles: [
  [[20, 40], [18, 42], [50, 90], [90, 50], [42, 18], [40, 20], [70, 50], [50, 70]]
]
```

### Obstacles intersect each other
![Example 4](/examples/example4.png)
```
x_start: 2
y_start: 2
x_goal: 98
y_goal: 98
x_space_size: 100
y_space_size: 100
list_obstacles: [
  [[40, 20], [40, 80], [60, 80], [60, 20]],
  [[20, 60], [80, 60], [80, 40], [20, 40]]
]
```