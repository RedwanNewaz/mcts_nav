#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <memory>
#include <limits>
#include "model/UnicycleAction.h"
#include "utility/ParamManager.h"
#include "env/CircularStaticObstacles.h"

class Node {
public:
    StatePtr state;
    std::shared_ptr<Node> parent;
    std::vector<std::shared_ptr<Node>> children;
    int visits;
    double value;

    Node(const StatePtr& state, std::shared_ptr<Node> parent = nullptr)
            : state(state), parent(parent), visits(0), value(0) {}
};

class MCTS {
private:
    std::shared_ptr<Node> root;
    int max_iterations;
    double exploration_weight;
    const int sim_count = 50;
    std::random_device rd;
    std::mt19937 gen;
    EnvPtr env_;

public:
    MCTS(const StatePtr& initial_state,
         EnvPtr  env,
         int max_iterations = 1000,
         double exploration_weight = 1.41):
              env_(std::move(env))
            , max_iterations(max_iterations)
            , exploration_weight(exploration_weight)
            , gen(rd()) {
        root = std::make_shared<Node>(initial_state);
    }

    StatePtr search() {
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

    bool is_terminal(const StatePtr& state) const {

        return env_->isTerminal(state) || env_->isCollision(state);
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
        auto action = get_untried_action(node);
        auto new_state = apply_action(node->state, action);
        auto child = std::make_shared<Node>(new_state, node);
        node->children.push_back(child);
        return child;
    }

    double simulate(const StatePtr& state) {
        StatePtr current_state = state;
        int count = sim_count;

        while (!is_terminal(current_state) && count > 0) {
            auto action = get_random_action();
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

    ActionPtr get_untried_action(std::shared_ptr<Node> node) {
        std::vector<std::pair<double, double>> tried_actions;
        for (const auto& child : node->children) {
            auto s = child->state->getArray();
            tried_actions.emplace_back(s[3], s[4]);
        }

        std::uniform_real_distribution<> v_dist(0, 1);
        std::uniform_real_distribution<> w_dist(-0.5, 0.5);

        double v, w;
        do {
            v = v_dist(gen);
            w = w_dist(gen);
        } while (std::find(tried_actions.begin(), tried_actions.end(),
                           std::make_pair(v, w)) != tried_actions.end());

        std::vector<double> u_range{0, 1, -0.5, 0.5};
        std::vector<double> u_res{0.1, 0.1};
        auto A0(std::make_shared<model::UnicycleAction>(u_range, u_res, v, w));
        return A0;
    }

    ActionPtr get_random_action() {
        std::uniform_real_distribution<> v_dist(0, 1);
        std::uniform_real_distribution<> w_dist(-0.5, 0.5);
        double v, w;
        v = v_dist(gen);
        w = w_dist(gen);
        std::vector<double> u_range{0, 1, -0.5, 0.5};
        std::vector<double> u_res{0.1, 0.1};
        auto A0(std::make_shared<model::UnicycleAction>(u_range, u_res, v, w));
        return A0;
    }

    StatePtr apply_action(const StatePtr& state, const ActionPtr& action) {
        const double dt = 0.1;
        auto s = state->getArray();
        auto u = action->getArray();
        double new_theta = s[2] + u[1] * dt;
        double new_x = s[0] + u[0] * std::cos(new_theta) * dt;
        double new_y = s[1] + u[0] * std::sin(new_theta) * dt;
        return std::make_shared<model::DiffWheelRobotState>(std::vector<double>{new_x, new_y, new_theta, u[0], u[1]}, state->getResolution());

    }



    double get_reward(const StatePtr& state) const {
        return env_->getReward(state);
    }
};

int main(int argc, char* argv[]) {

    param_manager pm(argv[1]);
    std::vector<std::vector<float>> OBS;
    pm.get_obstacles(OBS);
    auto goal = pm.get_param<std::vector<float>>("goal");
    auto start = pm.get_param<std::vector<float>>("start");
    auto robotRadius = pm.get_param<double>("robot_radius");
    auto goal_radius = pm.get_param<double>("goal_radius");
    auto env1(std::make_shared<env::StaticObstaclesEnv>(goal, OBS, robotRadius, goal_radius));


    std::vector<double> reso(5, robotRadius);
    std::vector<double> x(5, 0.0);
    x[0] = start[0];
    x[1] = start[1];
    x[2] = start[2];


    auto initial_state_ptr(std::make_shared<model::DiffWheelRobotState>(x, reso));


    for (int i = 0; i < 1000; ++i) {
        MCTS mcts(initial_state_ptr, env1, 30);
        auto best_state = mcts.search();

        std::cout << *best_state << std::endl;

        initial_state_ptr = std::dynamic_pointer_cast<model::DiffWheelRobotState>(best_state);
        if(mcts.is_terminal(initial_state_ptr))
        {
            printf("reached to terminal state\n");
            break;
        }

    }

    return 0;
}