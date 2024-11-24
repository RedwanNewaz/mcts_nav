//
// Created by airlab on 11/18/24.
//

#ifndef MCTS_MCTSPOLICY_H
#define MCTS_MCTSPOLICY_H
#include <memory>
#include "base/state.h"
#include "base/action.h"
#include "base/train.h"
#include "base/env.h"
#include "env/DynamicObstacles.h"
#include <random>
#include <future>
#include <unordered_set>
#define DEBUG(x) std::cout << "[MCTSPolicy]: " << x << std::endl

namespace mcts {
    // Add these constants to the class definition
    const double NEGATIVE_REWARD_THRESHOLD = -5;
    const int MIN_VISITS_BEFORE_PRUNING = 5;

    struct Node{
          StatePtr state;
          ActionPtr action;
          std::vector<std::shared_ptr<Node>> children;
          std::shared_ptr<Node> parent;
          int visits;
          double value;
          bool isTerminal;

        Node(const StatePtr& s, const ActionPtr& a, std::shared_ptr<Node> p= nullptr)
        :state(s), action(a), parent(p){
            visits = 0;
            value = 0.0;
            isTerminal = false;
        }

        double getUCT(double c = 1.41) const {
            if (visits == 0) return std::numeric_limits<double>::infinity();
            return (value / visits) + c * std::sqrt(std::log(parent->visits) / visits);
        }
        [[nodiscard]] std::size_t hash() const {
            std::size_t seed = state->hash();
            std::size_t a = action->hash();
            seed ^= std::hash<std::size_t>{}(a) + 0x9e3779b9 + (seed << 6) + (seed >> 2);

            return seed;
        }
        std::size_t operator()(const Node& obj) const {
            return obj.hash();
        }

        bool operator < (const Node& rhs) const {
            return getUCT(0) < rhs.getUCT(0);
        }

        bool operator > (const Node& rhs) const {
            return getUCT(0) > rhs.getUCT(0);
        }
    };
    using NodePtr = std::shared_ptr<Node>;
    class MCTSPolicy : public base::train{
    public:
        MCTSPolicy(const EnvPtr &env, const std::vector<double>& u_range,
                   const std::vector<double>& u_res, int numEpochs, const std::string &outfile="");

        NodePtr search(int max_iterations = 30);

        NodePtr getPolicy() const;

    private:
        EnvPtr env_;
        NodePtr root_;
        std::random_device rd;
        std::mt19937 gen;
        std::vector<double> u_range_;
        std::vector<double> u_res_;
        const int maxIterations_ = 50;
        const int maxSimSteps_ = 50;
        double explorationWeight = 1.67;

    protected:
        double step(int epoch) override;

        void save(const std::string &outfile) override;

    protected:
        NodePtr select(NodePtr node);
        NodePtr expand(NodePtr node);
        NodePtr bestChild(NodePtr node, double explorationWeight);
        double simulate(const StatePtr& state, int maxSimSteps = 50);
        void backpropagate(NodePtr node, double reward);
        StatePtr executeAction(const StatePtr& state, const ActionPtr& action);
        ActionPtr getUntriedAction(NodePtr node);
        bool shouldPruneNode(const NodePtr& node) const;


    };


} // mcts
namespace std {
    template<>
    struct hash<mcts::Node> {
        std::size_t operator()(const mcts::Node& obj) const {
            return obj.hash();
        }
    };
}

#endif //MCTS_MCTSPOLICY_H
