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
        self.distanceToGoal = float('inf')

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
    def get_config_distance(c1, c2, spec):
        max_ee1_delta = 0
        max_ee2_delta = 0
        for i in range(spec.num_segments):
            if abs((c2.ee1_angles[i] - c1.ee1_angles[i]).in_radians()) > max_ee1_delta:
                max_ee1_delta = abs((c2.ee1_angles[i] - c1.ee1_angles[i]).in_radians())

            if abs((c2.ee2_angles[i] - c1.ee2_angles[i]).in_radians()) > max_ee2_delta:
                max_ee2_delta = abs((c2.ee2_angles[i] - c1.ee2_angles[i]).in_radians())

        # measure leniently - allow compliance from EE1 or EE2
        max_delta = min(max_ee1_delta, max_ee2_delta)

        for i in range(spec.num_segments):
            if abs(c2.lengths[i] - c1.lengths[i]) > max_delta:
                max_delta = abs(c2.lengths[i] - c1.lengths[i])

        return max_delta

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

        if test_obstacle_collision(robotConfig, spec, spec.obstacles) and test_self_collision(robotConfig, spec):
            return robotConfig

        return generate_sample()

    def get_bridge_config(spec):
        pass

    def interpolate_path(RobotConfig1, RobotConfig2):
        paths = [RobotConfig1]

        lengths1 = RobotConfig1.lengths
        lengths2 = RobotConfig2.lengths

        kLengths = []
        for i in range(len(lengths1)):
            differences = lengths2[i] - lengths1[i]
            if differences >= 0:
                kLengths.append(math.floor(differences / 0.001))
            else:
                kLengths.append(math.ceil(differences / 0.001))

        angles1 = RobotConfig1.ee1_angles
        angles2 = RobotConfig2.ee1_angles

        kAngles = []
        for i in range(len(angles1)):
            differences = angles2[i].radians - angles1[i].radians
            if differences >= 0:
                kAngles.append(math.floor(differences / 0.001))
            else:
                kAngles.append(math.ceil(differences / 0.001))

        if RobotConfig1.ee1_grappled:
            ee2Grappled = True if RobotConfig1.ee2_grappled else False
            ee1Grappled = True
            grappledX, grappledY = RobotConfig1.get_ee1()
        else:
            ee2Grappled = True
            ee1Grappled = False
            grappledX, grappledY = RobotConfig1.get_ee2()

        #  for each config,
        newLengths = []
        newAngles = []
        counter = 0
        # getConfig2 = False
        while True:
            if counter < 2:
                counter += 1
            for i in range(len(lengths1)):
                if kLengths[i] > 0:
                    if counter == 1:
                        newLengths.append(lengths1[i] + 0.001)
                    else:
                        newLengths[i] += 0.001
                    kLengths[i] -= 1
                elif kLengths[i] < 0:
                    if counter == 1:
                        newLengths.append(lengths1[i] - 0.001)
                    else:
                        newLengths[i] -= 0.001
                    kLengths[i] += 1
                else:
                    if counter != 1:
                        if lengths2[i] - newLengths[i] > 0:
                            newLengths[i] += (lengths2[i] - newLengths[i])
                        elif lengths2[i] - newLengths[i] < 0:
                            newLengths[i] -= (lengths1[i] - lengths2[i])
                    else:
                        newLengths.append(lengths1[i])

                if kAngles[i] > 0:
                    if counter == 1:
                        newAngles.append(angles1[i] + 0.001)
                    else:
                        newAngles[i] += 0.001
                    kAngles[i] -= 1

                    print(newAngles[i])
                    print(angles2[i])
                elif kAngles[i] < 0:
                    if counter == 1:
                        newAngles.append(angles1[i] - 0.001)
                    else:
                        newAngles[i] -= 0.001
                    kAngles[i] += 1
                else:
                    if counter != 1:
                        # if i == 0:
                        #     print("This")
                        if angles2[i] > newAngles[i]:
                            newAngles[i] += (angles2[i] - newAngles[i])
                        elif angles2[i] < newAngles[i]:
                            newAngles[i] -= (newAngles[i] - angles2[i])
                    else:
                        newAngles.append(angles1[i])
                if i == len(lengths1) - 1:
                    newConfig = make_robot_config_from_ee1(x=grappledX, y=grappledY,
                                                           angles=newAngles.copy(), lengths=newLengths.copy(),
                                                           ee1_grappled=ee1Grappled, ee2_grappled=ee2Grappled)
                    paths.append(newConfig)

            if paths[-1]:
                if test_config_equality(paths[-1], RobotConfig2, spec):
                    break

        return paths

    def check_path_collision(RobotConfig1, RobotConfig2):
        """
        return: true if pass, false otherwise
        """
        configList = interpolate_path(RobotConfig1, RobotConfig2)
        for config in configList:
            if not test_obstacle_collision(config, spec, spec.obstacles):
                return False
        return True

    def connect_node(Node1, Node2):
        if check_path_collision(Node1.config, Node2.config):
            GraphNode.add_connection(Node1, Node2)

    def build_graph():
        graph = []
        paths = []
        # graph.add(init_node)
        # graph.add(goal_node)

        minInitDistance = float('inf')
        minGoalDistance = float('inf')
        while True:
            numberOfSamples = 3
            for i in range(numberOfSamples):
                randomConfig = generate_sample()

                # initDistance = get_config_distance(init_node.config, randomConfig, spec)
                # if initDistance < minInitDistance:
                #     minInitDistance = initDistance
                #     configNearInit = randomConfig
                #
                # goalDistance = get_config_distance(goal_node.config, randomConfig, spec)
                # if goalDistance < minGoalDistance:
                #     minGoalDistance = goalDistance
                #     configNearGoal = randomConfig

                newNode = GraphNode(spec, randomConfig)

                if graph:
                    for node in graph:
                        # Should bias the node with low distance
                        connect_node(newNode, node)
                        # careful
                    connect_node(newNode, goal_node)
                    connect_node(newNode, init_node)
                    graph.append(newNode)
                else:
                    graph.append(newNode)

            # connect init and goal to the graph
            # nodeNearInit = GraphNode(spec, configNearInit)
            # nodeNearGoal = GraphNode(spec, configNearGoal)
            # connect_node(init_node, nodeNearInit)
            # connect_node(goal_node, nodeNearGoal)

            initPaths = find_graph_path(spec, init_node)
            # if initPaths:
            #      return initPaths
            # else:
            #     continue

            foundGoal = True
            if initPaths:
                for i in range(len(initPaths) - 1):
                    if check_path_collision(initPaths[i], initPaths[i + 1]):
                        paths += (interpolate_path(initPaths[i], initPaths[i + 1]))
                    else:
                        foundGoal = False
                        paths = []
                        break
                if foundGoal:
                    return paths
            # else:
            #     print("Init path is None")

    if len(arglist) > 1:
        # counter = 0
        # while counter < 1000:
        #     counter += 1
        #     print(counter)
        #     config = generate_sample()
        #     steps.append(config)
        steps = build_graph()
        write_robot_config_list_to_file(output_file, steps)

    #
    # You may uncomment this line to launch visualiser once a solution has been found. This may be useful for debugging.
    # *** Make sure this line is commented out when you submit to Gradescope ***
    #
    v = Visualiser(spec, steps)


if __name__ == '__main__':
    main(sys.argv[1:])
