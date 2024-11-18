import matplotlib.pyplot as plt
from matplotlib.patches import Circle

class Simulation:
    def __init__(self, obstacles, goal):
        self.obstacles = obstacles
        self.goal_state = goal
        self.fig, self.ax = plt.subplots(figsize=(10, 10))

    def __call__(self, robot):
        self.robot = robot
        self.run()

    def run(self):
        self.ax.cla()
        for obstacle in self.obstacles:
            circ = Circle(obstacle, radius=0.5, color='black')
            self.ax.add_patch(circ)
        # plot goal
        circ = Circle((self.goal_state.x, self.goal_state.y), radius=0.5, color='green')
        self.ax.add_patch(circ)
        # plot robot
        circ = Circle((self.robot.x, self.robot.y), radius=0.5, color='red')
        self.ax.add_patch(circ)

        plt.axis([-2, 12, -2, 12])
        plt.pause(0.1)
