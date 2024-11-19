#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <memory>
#include <limits>

class State {
public:
    double x, y, theta, v, w;

    State(double x = 0, double y = 0, double theta = 0, double v = 0, double w = 0)
            : x(x), y(y), theta(theta), v(v), w(w) {}

    friend std::ostream& operator<<(std::ostream& os, const State& state) {
        os << "x: " << state.x << ", y: " << state.y;
        return os;
    }
};

class Action {
public:
    double v, w;

    Action(double v = 0, double w = 0) : v(v), w(w) {}
};

class Node {
public:
    State state;
    std::shared_ptr<Node> parent;
    std::vector<std::shared_ptr<Node>> children;
    int visits;
    double value;

    Node(const State& state, std::shared_ptr<Node> parent = nullptr)
            : state(state), parent(parent), visits(0), value(0) {}
};

class MCTS {
private:
    std::shared_ptr<Node> root;
    State goal_state;
    std::vector<std::pair<double, double>> obstacles;
    int max_iterations;
    double exploration_weight;
    const double robot_radius = 0.25;
    const double obstacle_radius = 0.25;
    const int sim_count = 50;
    const double goal_distance = 0.3;
    const double collision_distance;
    std::random_device rd;
    std::mt19937 gen;

public:
    MCTS(const State& initial_state,
         const State& goal_state,
         const std::vector<std::pair<double, double>>& obstacles,
         int max_iterations = 1000,
         double exploration_weight = 1.41)
            : goal_state(goal_state)
            , obstacles(obstacles)
            , max_iterations(max_iterations)
            , exploration_weight(exploration_weight)
            , collision_distance(2 * (robot_radius + obstacle_radius))
            , gen(rd()) {
        root = std::make_shared<Node>(initial_state);
    }

    State search() {
        for (int i = 0; i < max_iterations; ++i) {
            auto node = select(root);
            if (!is_terminal(node->state)) {
                auto child = expand(node);
                double reward = simulate(child->state);
                backpropagate(child, reward);
            }
        }
        return best_child(root, 0)->state;
    }

    bool is_terminal(const State& state) const {
        double goal_dist = std::sqrt(std::pow(state.x - goal_state.x, 2) +
                                     std::pow(state.y - goal_state.y, 2));
        if (goal_dist < goal_distance) {
            return true;
        }

        for (const auto& obstacle : obstacles) {
            double obstacle_dist = std::sqrt(std::pow(state.x - obstacle.first, 2) +
                                             std::pow(state.y - obstacle.second, 2));
            if (obstacle_dist <= collision_distance) {
                return true;
            }
        }
        return false;
    }

private:
    std::shared_ptr<Node> select(std::shared_ptr<Node> node) {
        while (!node->children.empty()) {
            if (node->children.size() < get_possible_actions()) {
                return expand(node);
            }
            node = best_child(node, exploration_weight);
        }
        return node;
    }

    std::shared_ptr<Node> expand(std::shared_ptr<Node> node) {
        Action action = get_untried_action(node);
        State new_state = apply_action(node->state, action);
        auto child = std::make_shared<Node>(new_state, node);
        node->children.push_back(child);
        return child;
    }

    double simulate(const State& state) {
        State current_state = state;
        int count = sim_count;

        while (!is_terminal(current_state) && count > 0) {
            Action action = get_random_action();
            current_state = apply_action(current_state, action);
            count--;
        }

        return get_reward(current_state);
    }

    void backpropagate(std::shared_ptr<Node> node, double reward) {
        while (node) {
            node->visits++;
            node->value += reward;
            node = node->parent;
        }
    }

    std::shared_ptr<Node> best_child(std::shared_ptr<Node> node, double exploration_weight) {
        auto ucb = [&](const std::shared_ptr<Node>& child) {
            double exploitation = child->value / (child->visits + 1.0);
            double exploration = std::sqrt(2 * std::log(node->visits + 1.0) / (child->visits + 1.0));
            return exploitation + exploration_weight * exploration;
        };

        return *std::max_element(node->children.begin(), node->children.end(),
                                 [&](const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) {
                                     return ucb(a) < ucb(b);
                                 });
    }

    int get_possible_actions() const {
        return 100; // 10 * 10 from v_range and w_range
    }

    Action get_untried_action(std::shared_ptr<Node> node) {
        std::vector<std::pair<double, double>> tried_actions;
        for (const auto& child : node->children) {
            tried_actions.emplace_back(child->state.v, child->state.w);
        }

        std::uniform_real_distribution<> v_dist(0, 1);
        std::uniform_real_distribution<> w_dist(-0.5, 0.5);

        double v, w;
        do {
            v = v_dist(gen);
            w = w_dist(gen);
        } while (std::find(tried_actions.begin(), tried_actions.end(),
                           std::make_pair(v, w)) != tried_actions.end());

        return Action(v, w);
    }

    Action get_random_action() {
        std::uniform_real_distribution<> v_dist(0, 1);
        std::uniform_real_distribution<> w_dist(-0.5, 0.5);
        return Action(v_dist(gen), w_dist(gen));
    }

    State apply_action(const State& state, const Action& action) {
        const double dt = 0.1;
        double new_theta = state.theta + action.w * dt;
        double new_x = state.x + action.v * std::cos(new_theta) * dt;
        double new_y = state.y + action.v * std::sin(new_theta) * dt;

        return State(new_x, new_y, new_theta, action.v, action.w);
    }



    double get_reward(const State& state) const {
        double goal_dist = std::sqrt(std::pow(state.x - goal_state.x, 2) +
                                     std::pow(state.y - goal_state.y, 2));

        if (goal_dist < goal_distance) {
            return 1.0e3;
        }

        for (const auto& obstacle : obstacles) {
            double obstacle_dist = std::sqrt(std::pow(state.x - obstacle.first, 2) +
                                             std::pow(state.y - obstacle.second, 2));
            if (obstacle_dist <= collision_distance) {
                return -1.0e3;
            }
        }

        return 1.0e3 * std::exp(-goal_dist);
    }
};

int main() {
    State initial_state(0, 0, 0, 0, 0);
    State goal_state(4, 10, 0, 0, 0);
    std::vector<std::pair<double, double>> obstacles = {
            {5, 5}, {7, 3}, {3, 7}
    };
    MCTS mcts(initial_state, goal_state, obstacles, 30);
    for (int i = 0; i < 1000; ++i) {

        State best_state = mcts.search();

        std::cout << "Best state: x=" << best_state.x << ", y=" << best_state.y
                  << ", theta=" << best_state.theta << std::endl;

        initial_state = best_state;
        if(mcts.is_terminal(initial_state))
        {
            printf("reached to terminal state\n");
            break;
        }

    }

    return 0;
}