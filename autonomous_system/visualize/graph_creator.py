import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import sys
sys.path.append('../')

from autonomous_system import AutonomousSystem as AS

SM = AS()
G = nx.DiGraph()

# nodes
for state in SM.states:
    G.add_node(state.name)

# edges
for transition in SM.transitions:
    source = transition.source.name
    destination = transition.destinations[0].name
    G.add_edge(source, destination)

pos = nx.spring_layout(G)

OPTIONS = {
    'node_color': 'skyblue',
    'node_size':10000,
    'width': 2,
    'with_labels': True
}

plt.figure(figsize=(8,8))

nx.draw(G, **OPTIONS)
#nx.draw_networkx_edge_labels(G,pos,edge_labels={
#     ('AS Off', 'Manual Driving'):'AB',
#     ('Manual Driving', 'AS Off'): 'BA'}, font_color='red')
plt.savefig("graph.png")
plt.show()