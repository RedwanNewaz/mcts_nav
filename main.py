from MCTS import MCTS
from State import State
from Simulation import Simulation


if __name__ == '__main__':
    # Example usage
    initial_state = State(0, 0, 0, 0, 0)
    goal_state = State(4, 10, 0, 0, 0)
    obstacles = [(5, 5), (7, 3), (3, 7)]  # List of obstacle positions



    sim = Simulation(obstacles, goal_state)
    for _ in range(1000):
        mcts = MCTS(initial_state, goal_state, obstacles, max_iterations=30)
        best_state = mcts.search()
        print(f"Best state: x={best_state.x:.4f}, y={best_state.y:.4f}, theta={best_state.theta:.4f}")
        initial_state = best_state
        sim(best_state)
        if mcts.is_terminal(initial_state):
            break