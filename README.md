# A\* Search in a Fantasy Game Map

This project demonstrates the **A* (A-star) search algorithm*\* in Python. The A\* algorithm is a widely-used pathfinding and graph traversal algorithm that finds the shortest path between a starting and a goal node in a graph. It is often used in video games, robotics, and other applications that require efficient pathfinding.

The provided code applies the A\* algorithm to find the optimal route through a fantasy-themed game map.

-----

## How it Works

The A\* algorithm evaluates a cost function for each node in the graph, which combines the cost from the start node and an estimated cost to the end node. This is represented by the formula:

$f(n) = g(n) + h(n)$

  * **`g(n)`**: The cost of the path from the start node to the current node `n`. In this project, this is the sum of the edge weights (travel costs) along the path.
  * **`h(n)`**: The **heuristic** cost, which is an estimated cost from node `n` to the goal node. For this project, a pre-defined dictionary provides these values, representing an approximation of the remaining distance to the goal.
  * **`f(n)`**: The total estimated cost of the path from the start to the goal, going through node `n`.

The algorithm explores nodes with the lowest `f(n)` value first, ensuring it prioritizes paths that seem most promising, leading to an efficient search for the optimal path.

-----

## Getting Started

### Prerequisites

You'll need Python installed on your system. This project also requires two external libraries: `networkx` and `matplotlib`.

You can install them using pip:

```bash
pip install networkx matplotlib
```

### Running the Code

1.  Save the provided Python script as `astar_search.py`.
2.  Open your terminal or command prompt.
3.  Navigate to the directory where you saved the file.
4.  Run the script using the following command:

<!-- end list -->

```bash
python astar_search.py
```

The program will print the calculated optimal path in the terminal and then display a graphical visualization of the game map with the optimal path highlighted.

-----

## The Game Map

The game map is represented as a dictionary called `game_map_graph`, where each key is a location (node) and the value is a list of its neighboring locations and the cost to travel there.

The `heuristics_game` dictionary provides the estimated cost (`h` value) from each location to the final destination, 'EndCity'.

Here's a breakdown of the map:

| Node           | Neighbors                                   | Heuristic to 'EndCity' |
| :------------- | :------------------------------------------ | :--------------------- |
| `StartVillage` | `ForestPath` (cost: 5), `RiverCrossing` (cost: 3) | 10                     |
| `ForestPath`   | `MountainPass` (cost: 5), `AncientRuins` (cost: 3) | 8                      |
| `RiverCrossing`| `AncientRuins` (cost: 2), `WizardTower` (cost: 4) | 6                      |
| `MountainPass` | `DragonLair` (cost: 6)                        | 7                      |
| `AncientRuins` | `DragonLair` (cost: 3), `EndCity` (cost: 5)     | 4                      |
| `DragonLair`   | `EndCity` (cost: 2)                           | 2                      |
| `WizardTower`  | `EndCity` (cost: 4)                           | 3                      |
| `EndCity`      | *None* | 0                      |

-----

## Code Structure

  * `Node` class: A simple class to represent a node in the graph, storing its name, parent, and `g`, `h`, and `f` costs.
  * `a_star_graph_search` function: This is the core of the A\* algorithm. It takes the graph, start and end nodes, and the heuristics as input. It uses a priority queue (`heapq`) to efficiently select the next node to explore.
  * `visualize_game_map` function: Uses `networkx` and `matplotlib` to create a visual representation of the graph. It highlights the nodes and edges that make up the optimal path found by the A\* algorithm.
  * `if __name__ == '__main__':`: The main execution block that sets up the problem, calls the search function, and then visualizes the result.
