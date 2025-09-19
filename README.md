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

## **Imports**

```python
import heapq
import networkx as nx
import matplotlib.pyplot as plt
```

* **heapq** → provides a priority queue (min-heap), used for managing nodes in the A\* open list.
* **networkx** → library for graph representation and manipulation.
* **matplotlib.pyplot** → for plotting and visualizing the graph.

---

## **Node Class**

```python
class Node:
    def __init__(self, name, parent=None):
        self.name = name
        self.parent = parent
        self.g = float('inf')   # Cost from start to this node
        self.h = 0              # Heuristic cost estimate to goal
        self.f = float('inf')   # Total cost (f = g + h)
```

* Represents a single location (node) in the graph.
* Each node stores:

  * **name**: the node’s unique identifier (e.g., `"StartVillage"`).
  * **parent**: the node from which it was reached (to reconstruct the path).
  * **g**: cost from start node to this node.
  * **h**: heuristic value (estimated distance to the goal).
  * **f**: total cost (actual cost + heuristic).

```python
    def __eq__(self, other):
        return self.name == other.name
```

* Equality check: two nodes are equal if their names are the same.

```python
    def __lt__(self, other):
        return self.f < other.f
```

* Less-than check: needed for heapq priority queue (compares nodes based on `f`).

```python
    def __hash__(self):
        return hash(self.name)
```

* Allows using `Node` objects in sets/dictionaries (uniquely identified by `name`).

---

## **A* Search Function*\*

```python
def a_star_graph_search(graph, start_name, end_name, heuristics):
    start_node = Node(start_name)
    end_node = Node(end_name)
    open_list = []
    node_map = {}
```

* Initializes start and end nodes.
* **open\_list**: priority queue (nodes waiting to be explored).
* **node\_map**: dictionary mapping node names → Node objects (so each name corresponds to a single node object).

---

### **Initialize Start Node**

```python
    start_node.g = 0
    start_node.h = heuristics.get(start_name, 0)
    start_node.f = start_node.g + start_node.h
    heapq.heappush(open_list, (start_node.f, start_node))
    node_map[start_name] = start_node
```

* Start node’s `g = 0` (cost from start to start is 0).
* Heuristic value is retrieved from the given dictionary.
* `f = g + h` is calculated.
* Push start node into the priority queue.
* Store it in `node_map`.

---

### **Closed List**

```python
    closed_list_names = set()
```

* Keeps track of already visited nodes (to avoid re-processing).

---

### **Main Loop**

```python
    while open_list:
        current_f, current_node = heapq.heappop(open_list)
```

* Continuously picks the node with the **lowest `f` value** (best candidate path).

```python
        if current_node.name in closed_list_names:
            continue
```

* Skip if this node was already processed.

```python
        closed_list_names.add(current_node.name)
```

* Mark current node as visited.

---

### **Goal Check**

```python
        if current_node.name == end_name:
            path = []
            current = current_node
            while current is not None:
                path.append(current)
                current = current.parent
            return path[::-1]
```

* If we reached the goal (`EndCity`), reconstruct the path by backtracking through `parent` links.
* Return the path in correct order (reversed list).

---

### **Expand Neighbors**

```python
        for neighbor_name, cost in graph.get(current_node.name, []):
            if neighbor_name in closed_list_names:
                continue
```

* Explore each connected neighbor of the current node.
* Skip if already visited.

```python
            new_g = current_node.g + cost
            new_h = heuristics.get(neighbor_name, 0)
            new_f = new_g + new_h
```

* Compute new costs:

  * `g`: actual path cost.
  * `h`: heuristic estimate.
  * `f = g + h`.

---

### **Add or Update Neighbor**

```python
            if neighbor_name not in node_map:
                neighbor_node = Node(neighbor_name, current_node)
                neighbor_node.g = new_g
                neighbor_node.h = new_h
                neighbor_node.f = new_f
                node_map[neighbor_name] = neighbor_node
                heapq.heappush(open_list, (neighbor_node.f, neighbor_node))
```

* If neighbor hasn’t been visited yet, create a new Node object.
* Set costs and push to open list.

```python
            else:
                neighbor_node = node_map[neighbor_name]
                if new_g < neighbor_node.g:
                    neighbor_node.g = new_g
                    neighbor_node.f = new_f
                    neighbor_node.parent = current_node
                    heapq.heappush(open_list, (neighbor_node.f, neighbor_node))
```

* If neighbor already exists but the new path is **cheaper**, update its cost and parent.
* Push updated node back into the open list.

---

### **If No Path**

```python
    return None
```

* If the loop ends with no solution, return `None`.

---

## **Graph Data (Fantasy Game Map)**

```python
game_map_graph = {
    'StartVillage': [('ForestPath', 5), ('RiverCrossing', 3)],
    'ForestPath': [('MountainPass', 5), ('AncientRuins', 3)],
    'RiverCrossing': [('AncientRuins', 2), ('WizardTower', 4)],
    'MountainPass': [('DragonLair', 6)],
    'AncientRuins': [('DragonLair', 3), ('EndCity', 5)],
    'DragonLair': [('EndCity', 2)],
    'WizardTower': [('EndCity', 4)],
    'EndCity': []
}
```

* Dictionary of nodes and their neighbors with travel cost.
* Example: `StartVillage → ForestPath (5)` means moving to `ForestPath` costs 5.

---

## **Heuristics (Estimated Distances to Goal)**

```python
heuristics_game = {
    'StartVillage': 10,
    'ForestPath': 8,
    'RiverCrossing': 6,
    'MountainPass': 7,
    'AncientRuins': 4,
    'DragonLair': 2,
    'WizardTower': 3,
    'EndCity': 0
}
```

* Provides a heuristic estimate of remaining distance to `EndCity`.
* A\* uses this to prioritize promising paths.

---

## **Visualization Function**

```python
def visualize_game_map(graph_data, optimal_path_nodes_obj=None):
    G = nx.DiGraph()
```

* Creates a directed graph.

```python
    for node, neighbors in graph_data.items():
        G.add_node(node)
        for neighbor, cost in neighbors:
            G.add_edge(node, neighbor, weight=cost)
```

* Adds nodes and edges (with weights).

```python
    pos = {
        'StartVillage': (0, 0),
        'ForestPath': (2, 2),
        'RiverCrossing': (2, -2),
        'MountainPass': (4, 3),
        'AncientRuins': (4, 0),
        'DragonLair': (6, 2),
        'WizardTower': (6, -2),
        'EndCity': (8, 0)
    }
```

* Assigns fixed positions for a nice layout.

```python
    node_colors = ['lightblue' for _ in G.nodes()]
    node_labels = {node: node for node in G.nodes()}
    edge_colors = ['gray' for _ in G.edges()]
    edge_widths = [1 for _ in G.edges()]
```

* Sets default styles for visualization.

---

### **Highlight Optimal Path**

```python
    if optimal_path_nodes_obj:
        path_node_names = [node_obj.name for node_obj in optimal_path_nodes_obj]
        path_edges = []
        for i in range(len(path_node_names) - 1):
            path_edges.append((path_node_names[i], path_node_names[i+1]))
```

* Extracts the optimal path node names and edges.

```python
        for i, node_name in enumerate(G.nodes()):
            if node_name in path_node_names:
                node_colors[list(G.nodes()).index(node_name)] = 'salmon'
```

* Highlight nodes in the path with **salmon color**.

```python
        for i, edge in enumerate(G.edges()):
            if edge in path_edges:
                edge_colors[i] = 'red'
                edge_widths[i] = 3
```

* Highlight path edges in **red** and make them thicker.

---

### **Draw Graph**

```python
    plt.figure(figsize=(12, 8))
    nx.draw_networkx_nodes(G, pos, node_color=node_colors, node_size=800)
    nx.draw_networkx_labels(G, pos, labels=node_labels, font_size=10, font_weight='bold')
    nx.draw_networkx_edges(G, pos, edge_color=edge_colors, width=edge_widths, arrowsize=20, alpha=0.7)
    edge_labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_color='darkgreen')

    plt.title("A* Search in a Fantasy Game Map")
    plt.axis('off')
    plt.show()
```

* Draws nodes, edges, labels, weights, and highlights the optimal path.

---

## **Main Execution**

```python
if __name__ == '__main__':
    start_location = 'StartVillage'
    end_location = 'EndCity'

    print(f"Finding optimal path from {start_location} to {end_location}...")
    optimal_path_nodes = a_star_graph_search(game_map_graph, start_location, end_location, heuristics_game)

    if optimal_path_nodes:
        path_names = [node_obj.name for node_obj in optimal_path_nodes]
        print(f"Optimal Path: {path_names}")
        print("\nVisualizing the path...")
        visualize_game_map(game_map_graph, optimal_path_nodes)
    else:
        print("No path found.")
```
### Demo

https://github.com/user-attachments/assets/8d642255-6911-49e5-ac69-2c2b8f2524bc


