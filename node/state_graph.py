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
        self.state = data.data
        if self.state != self.previous_state:
            self.previous_state = self.state
            self.color_graph()

    def add_nodes(self):
        for state in self.SM.states:
            self.G.add_node(state.name)

    def add_edges(self):
        for transition in self.SM.transitions:
            source = transition.source.name
            destination = transition.destinations[0].name
            self.G.add_edge(source, destination)

    def color_graph(self):
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

    def fig_to_array(self, fig):
        # draw the renderer
        fig.canvas.draw()

        # Get the RGB buffer from the figure
        w, h = fig.canvas.get_width_height()
        buf = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        buf.shape = (w, h, 3)

        return buf


if __name__ == '__main__':
    graph = Graph()
    while not rp.is_shutdown():
        rp.sleep(0.5)
