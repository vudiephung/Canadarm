import sys
import random

from tester import *
from problem_spec import ProblemSpec
from robot_config import *
from tester import test_config_equality
from visualiser import Visualiser

"""
Template file for you to implement your solution to Assignment 2. Contains a class you can use to represent graph nodes,
and a method for finding a path in a graph made up of GraphNode objects.

COMP3702 2020 Assignment 2 Support Code
"""


class GraphNode:
    """
    Class representing a node in the state graph. You should create an instance of this class each time you generate
    a sample.
    """

    def __init__(self, spec, config):
        """
        Create a new graph node object for the given config.

        Neighbors should be added by appending to self.neighbors after creating each new GraphNode.

        :param spec: ProblemSpec object
        :param config: the RobotConfig object to be stored in this node
        """
        self.spec = spec
        self.config = config
        self.neighbors = []

    def __eq__(self, other):
        return test_config_equality(self.config, other.config, self.spec)

    def __hash__(self):
        return hash(tuple(self.config.points))

    def get_successors(self):
        return self.neighbors

    @staticmethod
    def add_connection(n1, n2):
        """
        Creates a neighbor connection between the 2 given GraphNode objects.

        :param n1: a GraphNode object
        :param n2: a GraphNode object
        """
        n1.neighbors.append(n2)
        n2.neighbors.append(n1)


def find_graph_path(spec, init_node):
    """
    This method performs a breadth first search of the state graph and return a list of configs which form a path
    through the state graph between the initial and the goal. Note that this path will not satisfy the primitive step
    requirement - you will need to interpolate between the configs in the returned list.

    You may use this method in your solver if you wish, or can implement your own graph search algorithm to improve
    performance.

    :param spec: ProblemSpec object
    :param init_node: GraphNode object for the initial configuration
    :return: List of configs forming a path through the graph from initial to goal
    """
    # search the graph
    init_container = [init_node]

    # here, each key is a graph node, each value is the list of configs visited on the path to the graph node
    init_visited = {init_node: [init_node.config]}

    while len(init_container) > 0:
        current = init_container.pop(0)

        if test_config_equality(current.config, spec.goal, spec):
            # found path to goal
            return init_visited[current]

        successors = current.get_successors()
        for suc in successors:
            if suc not in init_visited:
                init_container.append(suc)
                init_visited[suc] = init_visited[current] + [suc.config]

    return None


def main(arglist):
    input_file = arglist[0]
    output_file = arglist[1]

    spec = ProblemSpec(input_file)

    init_node = GraphNode(spec, spec.initial)
    goal_node = GraphNode(spec, spec.goal)

    steps = []

    #
    #
    # Code for your main method can go here.
    #
    # Your code should find a sequence of RobotConfig objects such that all configurations are collision free, the
    # distance between 2 successive configurations is less than 1 primitive step, the first configuration is the initial
    # state and the last configuration is the goal state.
    #
    #

    def generate_sample():
        initialConfig = spec.initial

        initialAngles = initialConfig.ee1_angles
        minKAngle = -2879
        maxKAngle = 2879
        anglesRand = [Angle(radians=0.001) * random.randint(minKAngle, maxKAngle) for angleObj in initialAngles]
        # print([angle.in_degrees() for angle in anglesRand])

        minKLength = 0
        maxKLength = (spec.max_lengths[0] - spec.min_lengths[0]) / 0.001
        maxKLength = float(f'{maxKLength:.3f}')
        if not maxKLength == 0:
            lengthsRand = [float(f'{length + random.randint(minKLength, maxKLength) * 1e-3:.3f}')
                           for length in spec.min_lengths]
        else:
            lengthsRand = spec.min_lengths

        randomIndexGrapplePoint = random.randint(0, len(spec.grapple_points) - 1)
        grappleXRand, grappleYRand = spec.grapple_points[randomIndexGrapplePoint]

        # check number of grapple
        numGrapplePoints = spec.num_grapple_points
        if numGrapplePoints == 1:
            ee1_grappled = True if spec.initial.ee1_grappled else False
            ee2_grappled = not ee1_grappled
        else:
            ee1_grappled = bool(random.getrandbits(1))
            ee2_grappled = bool(random.getrandbits(1))

        if ee1_grappled:
            robotConfig = make_robot_config_from_ee1(grappleXRand, grappleYRand, anglesRand, lengthsRand,
                                                     ee1_grappled=ee1_grappled, ee2_grappled=ee2_grappled)
        else:
            robotConfig = make_robot_config_from_ee2(grappleXRand, grappleYRand, anglesRand, lengthsRand,
                                                     ee1_grappled=ee1_grappled, ee2_grappled=ee2_grappled)

        if test_obstacle_collision(robotConfig, spec, spec.obstacles):
            return robotConfig

        return generate_sample()

    def interpolate_path(RobotConfig1, RobotConfig2):
        pass

    def build_graph():
        graph = set()
        while True:
            for i in range(20):
                randomConfig = generate_sample()
                if not test_obstacle_collision(randomConfig, spec, spec.obstacles):
                    newNode = GraphNode(spec, randomConfig)
                    graph.add(newNode)
                    # Connect strategies
            # Search graph
            # if found: break

    if len(arglist) > 1:
        generate_sample()
        write_robot_config_list_to_file(output_file, steps)

    #
    # You may uncomment this line to launch visualiser once a solution has been found. This may be useful for debugging.
    # *** Make sure this line is commented out when you submit to Gradescope ***
    #
    # v = Visualiser(spec, steps)


if __name__ == '__main__':
    main(sys.argv[1:])
