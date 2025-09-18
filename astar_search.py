import heapq
import networkx as nx
import matplotlib.pyplot as plt


class Node:
    def __init__(self, name, parent=None):
        self.name = name
        self.parent = parent
        self.g = float('inf')
        self.h = 0
        self.f = float('inf')

    def __eq__(self, other):
        return self.name == other.name

    def __lt__(self, other):
        return self.f < other.f

    def __hash__(self):
        return hash(self.name)


def a_star_graph_search(graph, start_name, end_name, heuristics):
    start_node = Node(start_name)
    end_node = Node(end_name)
    open_list = []
    node_map = {}

    start_node.g = 0
    start_node.h = heuristics.get(start_name, 0)
    start_node.f = start_node.g + start_node.h
    heapq.heappush(open_list, (start_node.f, start_node))
    node_map[start_name] = start_node

    closed_list_names = set()

    while open_list:
        current_f, current_node = heapq.heappop(open_list)
        
        if current_node.name in closed_list_names:
            continue
            
        closed_list_names.add(current_node.name)

        if current_node.name == end_name:
            path = []
            current = current_node
            while current is not None:
                path.append(current)
                current = current.parent
            return path[::-1]

        for neighbor_name, cost in graph.get(current_node.name, []):
            if neighbor_name in closed_list_names:
                continue

            new_g = current_node.g + cost
            new_h = heuristics.get(neighbor_name, 0)
            new_f = new_g + new_h

            if neighbor_name not in node_map:
                neighbor_node = Node(neighbor_name, current_node)
                neighbor_node.g = new_g
                neighbor_node.h = new_h
                neighbor_node.f = new_f
                node_map[neighbor_name] = neighbor_node
                heapq.heappush(open_list, (neighbor_node.f, neighbor_node))
            else:
                neighbor_node = node_map[neighbor_name]
                if new_g < neighbor_node.g:
                    neighbor_node.g = new_g
                    neighbor_node.f = new_f
                    neighbor_node.parent = current_node
                    heapq.heappush(open_list, (neighbor_node.f, neighbor_node))

    return None

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
def visualize_game_map(graph_data, optimal_path_nodes_obj=None):
    G = nx.DiGraph()
    
    for node, neighbors in graph_data.items():
        G.add_node(node)
        for neighbor, cost in neighbors:
            G.add_edge(node, neighbor, weight=cost)

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
    node_colors = ['lightblue' for _ in G.nodes()]
    node_labels = {node: node for node in G.nodes()}
    edge_colors = ['gray' for _ in G.edges()]
    edge_widths = [1 for _ in G.edges()]

    if optimal_path_nodes_obj:
        path_node_names = [node_obj.name for node_obj in optimal_path_nodes_obj]
        path_edges = []
        for i in range(len(path_node_names) - 1):
            path_edges.append((path_node_names[i], path_node_names[i+1]))

        for i, node_name in enumerate(G.nodes()):
            if node_name in path_node_names:
                node_colors[list(G.nodes()).index(node_name)] = 'salmon'
                
        for i, edge in enumerate(G.edges()):
            if edge in path_edges:
                edge_colors[i] = 'red'
                edge_widths[i] = 3

    plt.figure(figsize=(12, 8))

    nx.draw_networkx_nodes(G, pos, node_color=node_colors, node_size=800)
    nx.draw_networkx_labels(G, pos, labels=node_labels, font_size=10, font_weight='bold')

    # Draw edges and edge weights
    nx.draw_networkx_edges(G, pos, edge_color=edge_colors, width=edge_widths, arrowsize=20, alpha=0.7)
    edge_labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_color='darkgreen')

    plt.title("A* Search in a Fantasy Game Map")
    plt.axis('off')
    plt.show()


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
