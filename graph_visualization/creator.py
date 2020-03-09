import matplotlib.pyplot as plt
import networkx as nx

G = nx.Graph()

# nodes
G.add_node('AS off')
G.add_node('AS manual')
G.add_node('AS ready')
G.add_node('AS driving')
G.add_node('AS finished')
G.add_node('AS emergancy')

# edges
G.add_edge('AS off', 'AS manual')
G.add_edge('AS manual', 'AS off')

G.add_edge('AS off', 'AS ready')
G.add_edge('AS ready', 'AS driving')
G.add_edge('AS driving', 'AS finished')
G.add_edge('AS finished', 'AS off')

G.add_edge('AS ready', 'AS emergancy')
G.add_edge('AS driving', 'AS emergancy')
G.add_edge('AS finished', 'AS emergancy')


OPTIONS = {
    'node_color': 'skyblue',
    'node_size': 500,
    'width': 1,
    'with_labels': True
}

plt.figure(figsize=(10, 10))
nx.draw(G, **OPTIONS)
plt.savefig("graph.png")
