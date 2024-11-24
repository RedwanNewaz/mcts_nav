//
// Created by airlab on 11/18/24.
//

#ifndef MCTS_MCTSPOLICY_H
#define MCTS_MCTSPOLICY_H

#include "env/DynamicObstacles.h"
#include <random>
#include <future>
#include <unordered_set>
#include "MCTSNode.h"
#define DEBUG(x) std::cout << "[MCTSPolicy]: " << x << std::endl

namespace mcts {
    // Add these constants to the class definition
    const double NEGATIVE_REWARD_THRESHOLD = -5;
    const int MIN_VISITS_BEFORE_PRUNING = 5;

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
