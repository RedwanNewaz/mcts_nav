import numpy as np
import matplotlib.pyplot as plt

def create_obstacles(sim_time, num_timesteps):
    # Obstacle 1
    v = -2
    p0 = np.array([5, 12])
    obst = create_robot(p0, v, np.pi/2, sim_time,
                        num_timesteps).reshape(4, num_timesteps, 1)
    obstacles = obst
    # Obstacle 2
    v = 2
    p0 = np.array([0, 5])
    obst = create_robot(p0, v, 0, sim_time, num_timesteps).reshape(
        4, num_timesteps, 1)
    obstacles = np.dstack((obstacles, obst))
    # Obstacle 3
    v = 2
    p0 = np.array([10, 10])
    obst = create_robot(p0, v, -np.pi * 3 / 4, sim_time, num_timesteps).reshape(4,
                                                                                num_timesteps, 1)
    obstacles = np.dstack((obstacles, obst))
    # Obstacle 4
    v = 2
    p0 = np.array([7.5, 2.5])
    obst = create_robot(p0, v, np.pi * 3 / 4, sim_time, num_timesteps).reshape(4,
                                                                               num_timesteps, 1)
    obstacles = np.dstack((obstacles, obst))

    return obstacles


def create_robot(p0, v, theta, sim_time, num_timesteps):
    # Creates obstacles starting at p0 and moving at v in theta direction
    t = np.linspace(0, sim_time, num_timesteps)
    theta = theta * np.ones(np.shape(t))
    vx = v * np.cos(theta)
    vy = v * np.sin(theta)
    v = np.stack([vx, vy])
    p0 = p0.reshape((2, 1))
    p = p0 + np.cumsum(v, axis=1) * (sim_time / num_timesteps)
    p = np.concatenate((p, v))
    return p


if __name__ == '__main__':
    SIM_TIME = 5.
    TIMESTEP = 0.1
    NUMBER_OF_TIMESTEPS = int(SIM_TIME / TIMESTEP)
    ROBOT_RADIUS = 0.5
    VMAX = 2
    VMIN = 0.2
    obstacles = create_obstacles(SIM_TIME, NUMBER_OF_TIMESTEPS)
    print(obstacles.shape)

    #TODO
    start = np.array([5, 0, 0, 0])
    goal = np.array([5, 10, 0, 0])

    for i in range(NUMBER_OF_TIMESTEPS):
        plt.clf()
        plt.scatter(obstacles[0, i, :], obstacles[1, i, :])
        plt.axis([-10, 20, -10, 20])
        plt.pause(0.1)


