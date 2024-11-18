import math
import random
import numpy as np
from Node import Node
from Action import Action
from State import State
class MCTS:
    def __init__(self, initial_state, goal_state, obstacles, max_iterations=1000, exploration_weight=1.41):
        self.root = Node(initial_state)
        self.goal_state = goal_state
        self.obstacles = obstacles
        self.max_iterations = max_iterations
        self.exploration_weight = exploration_weight
        self.robot_radius = 0.25
        self.obstacle_radius = 0.25
        self.sim_count = 50
        self.goal_distance = 0.3
        self.collision_distance = 2 * (self.robot_radius  + self.obstacle_radius)

    def search(self):
        for _ in range(self.max_iterations):
            node = self.select(self.root)
            if not self.is_terminal(node.state):
                child = self.expand(node)
                reward = self.simulate(child.state)
                self.backpropagate(child, reward)
        return self.best_child(self.root, 0).state

    def select(self, node):
        while node.children:
            if len(node.children) < self.get_possible_actions(node.state):
                return self.expand(node)
            node = self.best_child(node, self.exploration_weight)
        return node

    def expand(self, node):
        action = self.get_untried_action(node)
        new_state = self.apply_action(node.state, action)
        child = Node(new_state, parent=node)
        node.children.append(child)
        return child

    def simulate(self, state):
        count = self.sim_count
        while not self.is_terminal(state):
            action = self.get_random_action(state)
            state = self.apply_action(state, action)
            count -= 1
            if count <= 0:
                break
        return self.get_reward(state)

    def backpropagate(self, node, reward):
        while node:
            node.visits += 1
            node.value += reward
            node = node.parent

    def best_child(self, node, exploration_weight):
        def ucb(child):
            exploitation = child.value / (child.visits + 1.0)
            exploration = math.sqrt(2 * math.log(node.visits + 1.0) / (child.visits + 1.0))
            return exploitation + exploration_weight * exploration

        return max(node.children, key=ucb)

    def get_possible_actions(self, state):
        # Define possible actions based on velocity and angular velocity ranges
        v_range = np.linspace(0, 1, 10)  # Linear velocity range
        w_range = np.linspace(-0.5, 0.5, 10)  # Angular velocity range
        return len(v_range) * len(w_range)

    def get_untried_action(self, node):
        tried_actions = set((child.state.v, child.state.w) for child in node.children)
        v_range = np.linspace(0, 1, 10)
        w_range = np.linspace(-0.5, 0.5, 10)
        untried_actions = [(v, w) for v in v_range for w in w_range if (v, w) not in tried_actions]
        return Action(*random.choice(untried_actions))

    def get_random_action(self, state):
        v = random.uniform(0, 1)
        w = random.uniform(-0.5, 0.5)
        return Action(v, w)

    def apply_action(self, state, action):
        dt = 0.1  # Time step
        new_theta = state.theta + action.w * dt
        new_x = state.x + action.v * math.cos(new_theta) * dt
        new_y = state.y + action.v * math.sin(new_theta) * dt

        return State(new_x, new_y, new_theta, action.v, action.w)

    def is_terminal(self, state):
        # Check if the robot has reached the goal or collided with an obstacle
        goal_distance = math.sqrt((state.x - self.goal_state.x)**2 + (state.y - self.goal_state.y)**2)
        if goal_distance < self.goal_distance:
            return True
        for obstacle in self.obstacles:
            obstacle_distance = math.sqrt((state.x - obstacle[0])**2 + (state.y - obstacle[1])**2)
            if obstacle_distance <= self.collision_distance:  # Assuming obstacle radius is 0.5
                return True
        return False

    def get_reward(self, state):
        goal_distance = math.sqrt((state.x - self.goal_state.x)**2 + (state.y - self.goal_state.y)**2)
        if goal_distance < self.goal_distance:
            return 1.0e3  # Reached the goal
        for obstacle in self.obstacles:
            obstacle_distance = math.sqrt((state.x - obstacle[0])**2 + (state.y - obstacle[1])**2)
            if obstacle_distance <= self.collision_distance:
                return -1.0e3  # Collided with an obstacle
        return 1.0e3 * np.exp(-goal_distance) # Negative distance to goal as reward