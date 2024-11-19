//
// Created by airlab on 11/18/24.
//
#include <memory>
#include "algo/MCTSPolicy.h"
#include "model/UnicycleAction.h"
#include "model/DiffWheelRobotState.h"

#define UNTRYVER1


namespace mcts {
    MCTSPolicy::MCTSPolicy(const StatePtr& initX, const EnvPtr &env, const std::vector<double>& u_range,
                           const std::vector<double>& u_res, int numEpochs, const std::string &outfile)
    : train(numEpochs, outfile), u_range_(u_range), u_res_(u_res),
    env_(env), gen(rd())  {

        auto A0(std::make_shared<model::UnicycleAction>(u_range, u_res));
        root_ = std::make_shared<Node>(initX, A0);
    }

    NodePtr MCTSPolicy::select(NodePtr node) {
        auto action = std::dynamic_pointer_cast<model::UnicycleAction>(node->action);
        while (!node->children.empty()) {
            if (node->children.size() < 100) {
                return expand(node);
            }
            node = bestChild(node, 1.41);
        }
        return node;
    }

    NodePtr MCTSPolicy::expand(NodePtr node) {
        auto actionPtr = getUntriedAction(node);
        auto newStatePtr = executeAction(node->state, actionPtr);
        auto child = std::make_shared<Node>(newStatePtr, actionPtr, node);
        node->children.emplace_back(child);
        return child;
    }
#ifndef UNTRYVER1
    ActionPtr MCTSPolicy::getUntriedAction(NodePtr node) {

        std::vector<std::pair<double, double>> tried_actions;
        for (const auto& child : node->children) {
            auto act = child->action->getArray();
            tried_actions.emplace_back(act[0], act[1]);
        }

        std::uniform_real_distribution<> v_dist(u_range_[0], u_range_[1]);
        std::uniform_real_distribution<> w_dist(u_range_[2], u_range_[3]);

        double v, w;
        do {
            v = v_dist(gen);
            w = w_dist(gen);
        } while (std::find(tried_actions.begin(), tried_actions.end(),
                           std::make_pair(v, w)) != tried_actions.end());


        auto selectedAction =std::make_shared<model::UnicycleAction>(u_range_, u_res_, v, w);

        return selectedAction;

    }
#else
    ActionPtr MCTSPolicy::getUntriedAction(NodePtr node) {
        auto action = std::dynamic_pointer_cast<model::UnicycleAction>(node->action);
        auto allActions = action->getAllPossibleActions();

        // get get_untried_action
        std::vector<ActionPtr> untriedActions;
        for(auto& pact: allActions)
        {
            bool match = false;
            for (auto& child: node->children)
                if(pact == *child->action)
                {
                    match = true;
                    break;
                }
            if(match) continue;
            untriedActions.emplace_back(std::make_shared<model::UnicycleAction>(pact));
        }
        return action->sampleRandomActions(untriedActions, 1)[0];
    }
#endif

    StatePtr MCTSPolicy::executeAction(const StatePtr &state, const ActionPtr &action) {
        double dt_ = 0.1;
        auto s = state->getArray();
        auto u = action->getArray();
        double theta = s[2] + u[1] * dt_;
        double x = s[0] + u[0] * cos(theta) * dt_;
        double y = s[1] + u[0] * sin(theta) * dt_;
        std::vector<double>NewState{x, y, theta, u[0], u[1]};
        return std::make_shared<model::DiffWheelRobotState>(NewState, state->getResolution());
    }

    void MCTSPolicy::backpropagate(NodePtr node, double reward) {
        while(node != nullptr)
        {
            node->visits += 1;
            node->value += reward;
            node = node->parent;
        }

    }

#ifndef SIMVER1
    double MCTSPolicy::simulate(const StatePtr &state) {
        //TODO make it as a parameter
        int maxSimSteps = 50;
        auto newstate = state;


        while (env_->isTerminal(newstate) || maxSimSteps > 0)
        {
            std::uniform_real_distribution<> v_dist(u_range_[0], u_range_[1]);
            std::uniform_real_distribution<> w_dist(u_range_[2], u_range_[3]);
            double v = v_dist(gen);
            double w = w_dist(gen);
            auto selectedAction =std::make_shared<model::UnicycleAction>(u_range_, u_res_, v, w);
            newstate = executeAction(newstate, selectedAction);
            --maxSimSteps;
        }
        return env_->getReward(newstate);
    }
#else
    double MCTSPolicy::simulate(const StatePtr &state) {
        //TODO make it as a parameter
        int maxSimSteps = 50;
        auto newstate = state;
        auto action = std::dynamic_pointer_cast<model::UnicycleAction>(root_->action);
        auto allActions = action->getAllPossibleActions();

        while (env_->isTerminal(newstate) || maxSimSteps > 0)
        {
            auto act = action->sampleRandomActions(allActions, 1)[0];
            auto actPtr = std::make_shared<model::UnicycleAction>(act);
            newstate = executeAction(newstate, actPtr);
            --maxSimSteps;
        }

        return env_->getReward(newstate);
    }
#endif

    double MCTSPolicy::step() {
        double episodeReward = 0.0;
        while(!env_->isTerminal(root_->state))
        {
            auto node = search();
            episodeReward += env_->getReward(node->state);
//            DEBUG( "Distance: " << env_->goalDistance(node->state) <<" | " << episodeReward);
            root_ = node;
        }

        // reset root
        while(root_->parent)
            root_ = root_->parent;

        return episodeReward;
    }

    void MCTSPolicy::save(const std::string &outfile) {

    }

    NodePtr MCTSPolicy::bestChild(NodePtr node, double explorationWeight) {

        while (!node->children.empty()) {
            double bestValue = -std::numeric_limits<double>::infinity();
            NodePtr bestChild = nullptr;

            for (const auto& child : node->children) {
                double uctValue = child->getUCT(explorationWeight);
                if (uctValue > bestValue) {
                    bestValue = uctValue;
                    bestChild = child;
                }
            }
            node = bestChild;
        }
        return node;
    }

    NodePtr MCTSPolicy::search() {
        int max_iterations = 30;
        for (int i = 0; i < max_iterations; ++i) {
            auto node = select(root_);
            if(!env_->isTerminal(node->state) || !env_->isCollision(node->state))
            {
                auto child = expand(node);
                auto reward = simulate(child->state);
//                printf("reward =  %lf\n", reward);
                backpropagate(child, reward);
            }
        }
        return bestChild(root_, 0.0);
    }
} // mcts