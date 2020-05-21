#!/usr/bin/env python

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from autonomous_system import AutonomousSystem as AS

import rospy as rp
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Graph:
    """
    A class used to handle String messages from /state topic with current state name, 
    create state machine visualization and publish it with /state_graph topic as Image message.

    ...

    Attributes
    ----------
    state : str
        Autonomous System State Machine current state name.

    previous_state : str
        Autonomous System State Machine previous state name.

    SM : autonomous_system.AutonomousSystem class instance
        Instance of AutonomousSystem class which contain implementation of Autonomous System State Machine.

    G : networkx.DiGraph class instance
        Instance of DiGraph class for Autonomous System State Machine graph visualization.

    graph_options : dict
        DiGraph visualization parameters.

    pos : dict
        DiGraph Autonomous System State Machine nodes position, 
        needed for immutability of graph nodes position in visualization.

    state_graph_pub: rospy.Publisher
        Image message publisher of graph state visualization.    

    Methods
    -------
    state_callback(data)
        Get String data from subscribed /state topic and check if state changed.

    add_nodes()
        Add state nodes from Autonomous System State Machine to DiGraph.

    add_edges()
        Add transitions between nodes of Autonomous System State Machine to DiGraph.

    fig_to_array(fig)
        Change matplotlib figure to numpy array.
        
    color_graph()
        Color nodes of graph and publish /state_graph topic with Image message which contain
        visualization of Autonomous System State Machine with selected current state.
    """

    def __init__(self):
        self.state = None
        self.previous_state = None
        self.SM = AS()
        self.G = nx.DiGraph()
        self.add_nodes()
        self.add_edges()
        self.graph_options = {
            'node_size': 10000,
            'width': 2,
            'with_labels': True
        }

        self.pos = nx.spring_layout(self.G)

        rp.init_node('state_graph')
        rp.Subscriber('state', String, self.state_callback)

        self.state_graph_pub = rp.Publisher('state_graph', Image, queue_size=5)

    def state_callback(self, data):
        """Get String data from subscribed /state topic and check if state changed.

        Parameters
        ----------
        data : str
            Current state name.
        """

        self.state = data.data
        if self.state != self.previous_state:
            self.previous_state = self.state
            self.color_graph()

    def add_nodes(self):
        """Add state nodes from Autonomous System State Machine to DiGraph."""

        for state in self.SM.states:
            self.G.add_node(state.name)

    def add_edges(self):
        """Add transitions between nodes of Autonomous System State Machine to DiGraph."""

        for transition in self.SM.transitions:
            source = transition.source.name
            destination = transition.destinations[0].name
            self.G.add_edge(source, destination)

    def fig_to_array(self, fig):
        """Change matplotlib figure to numpy array.

        Parameters
        ----------
        fig : matplotlib.pyplot.figure
            Matplotlib figure.

        Returns
        -------
        buf : numpy.ndarray
            Numpy array wchich contain matplotlib figure.
        """

        # draw the renderer
        fig.canvas.draw()

        # Get the RGB buffer from the figure
        w, h = fig.canvas.get_width_height()
        buf = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        buf.shape = (w, h, 3)

        return buf

    def color_graph(self):
        """Color nodes of graph and publish /state_graph topic with Image message which contain
            visualization of Autonomous System State Machine with selected current state."""

        color_map = []
        for node in self.G:
            if node == self.state:
                color_map.append('red')
            else:
                color_map.append('skyblue')

        figure = plt.figure(figsize=(8, 8))

        nx.draw(self.G, pos=self.pos, node_color=color_map, **self.graph_options)

        img = self.fig_to_array(figure)

        bridge = CvBridge()
        img_msg = bridge.cv2_to_imgmsg(img, 'bgr8')

        self.state_graph_pub.publish(img_msg)


if __name__ == '__main__':
    graph = Graph()
    while not rp.is_shutdown():
        rp.sleep(0.5)
