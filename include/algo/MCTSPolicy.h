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
#include <random>
#define DEBUG(x) std::cout << "[MCTSPolicy]: " << x << std::endl

namespace mcts {
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
    };
    using NodePtr = std::shared_ptr<Node>;
    class MCTSPolicy : public base::train{
    public:
        MCTSPolicy(const StatePtr& initX, const EnvPtr &env, const std::vector<double>& u_range,
                   const std::vector<double>& u_res, int numEpochs, const std::string &outfile="");

        NodePtr search();

    private:
        EnvPtr env_;
        NodePtr root_;
        std::random_device rd;
        std::mt19937 gen;
        std::vector<double> u_range_;
        std::vector<double> u_res_;

    protected:
        double step() override;

        void save(const std::string &outfile) override;

    protected:
        NodePtr select(NodePtr node);
        NodePtr expand(NodePtr node);
        NodePtr bestChild(NodePtr node, double explorationWeight);
        double simulate(const StatePtr& state);
        void backpropagate(NodePtr node, double reward);
        StatePtr executeAction(const StatePtr& state, const ActionPtr& action);
        ActionPtr getUntriedAction(NodePtr node);


    };


} // mcts

#endif //MCTS_MCTSPOLICY_H
