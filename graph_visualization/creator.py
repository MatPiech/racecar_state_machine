import matplotlib.pyplot as plt
import networkx as nx

G = nx.DiGraph()

# nodes
G.add_node('AS off')
G.add_node('AS manual')
G.add_node('AS ready')
G.add_node('AS driving')
G.add_node('AS finished')
G.add_node('AS emergancy', weight='weight')

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
G.add_edge('AS emergancy', 'AS off')

pos = nx.spring_layout(G)

OPTIONS = {
    'node_color': 'skyblue',
    'node_size':5000,
    'width': 2,
    'with_labels': True
}

plt.figure(figsize=(8,8))

nx.draw(G, **OPTIONS)
# nx.draw_networkx_edge_labels(G,pos,edge_labels={
#     ('AS off', 'AS manual'):'AB',
#     ('AS manual', 'AS off'): 'BA'

# },font_color='red')
plt.savefig("graph.png")