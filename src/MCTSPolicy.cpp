//
// Created by airlab on 11/18/24.
//
#include <memory>
#include "algo/MCTSPolicy.h"
#include "model/UnicycleAction.h"
#include "model/DiffWheelRobotState.h"

#define UNTRYVER1
//#define ALL_POSSIBLE


namespace mcts {
    MCTSPolicy::MCTSPolicy(const EnvPtr &env, const std::vector<double>& u_range,
                           const std::vector<double>& u_res, int numEpochs, const std::string &outfile)
    : train(numEpochs, outfile), u_range_(u_range), u_res_(u_res),
    env_(env), gen(rd())  {
        auto initX = env->reset();
        auto A0(std::make_shared<model::UnicycleAction>(u_range, u_res));
        root_ = std::make_shared<Node>(initX, A0);
    }

    NodePtr MCTSPolicy::select(NodePtr node) {
        auto action = std::dynamic_pointer_cast<model::UnicycleAction>(node->action);
        while (!node->children.empty()) {
            if (node->children.size() < action->allPossibleActionSize()) {
                return expand(node);
            }
            node = bestChild(node, explorationWeight);
        }
        return node;
    }

    NodePtr MCTSPolicy::expand(NodePtr node) {

        // check if this child already exist
        bool isValidChild;
        NodePtr child;
        do{
            isValidChild = true;
            auto actionPtr = getUntriedAction(node);
            auto newStatePtr = executeAction(node->state, actionPtr);
            child = std::make_shared<Node>(newStatePtr, actionPtr, node);
            for(auto& ch: node->children)
            {
                if (*ch->state == *child->state && *ch->action == *child->action)
                {
                    isValidChild = false;
                    break;
                }
            }

        }while(!isValidChild);

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
//        printf("node size %zu \n", node->children.size());
        double v, w;
        do {
            v = v_dist(gen);
            w = w_dist(gen);
        } while (std::find(tried_actions.begin(), tried_actions.end(),
                           std::make_pair(v, w)) != tried_actions.end());


        auto selectedAction =std::make_shared<model::UnicycleAction>(u_range_, u_res_, v, w);

        return selectedAction;

    }
#elifdef EXHAUST
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
#elifdef ALL_POSSIBLE
    ActionPtr MCTSPolicy::getUntriedAction(NodePtr node) {

        // keep track of visited action set
        std::unordered_set<base::Action> triedActions;
        for(auto& child: node->children) {
            base::Action ca = *child->action;
            triedActions.insert(ca);
        }

        // create list of all possible actions
        auto action = std::dynamic_pointer_cast<model::UnicycleAction>(node->action);
        std::shared_ptr<model::UnicycleAction> selectedAction;
        auto ACTIONS = action->getAllPossibleActions();
        std::uniform_int_distribution<> a_dist(0, ACTIONS.size() - 1);


        // sample an action that never visited before
        for(int i = 0; i < action->allPossibleActionSize(); ++i) {
            auto u = ACTIONS[a_dist(gen)].getArray();
            selectedAction =std::make_shared<model::UnicycleAction>(u_range_, u_res_, u[0], u[1]);

            // found new action?
            if(triedActions.count(*selectedAction) == 0)
                break;
        };

        return selectedAction;
    }
#else
    ActionPtr MCTSPolicy::getUntriedAction(NodePtr node) {

        // keep track of visited action set
        std::unordered_set<base::Action> triedActions;
        for(auto& child: node->children) {
            base::Action ca = *child->action;
            triedActions.insert(ca);
        }

        // create list of all possible actions
        std::shared_ptr<model::UnicycleAction> selectedAction;
        std::uniform_real_distribution<> v_dist(u_range_[0], u_range_[1]);
        std::uniform_real_distribution<> w_dist(u_range_[2], u_range_[3]);
        std::array<double, 2> u;
        auto action = std::dynamic_pointer_cast<model::UnicycleAction>(node->action);

        // sample an action that never visited before
        for(int i = 0; i < action->allPossibleActionSize(); ++i) {
            u[0] = v_dist(gen);
            u[1] = w_dist(gen);
            selectedAction =std::make_shared<model::UnicycleAction>(u_range_, u_res_, u[0], u[1]);
            // found new action?
            if(triedActions.count(*selectedAction) == 0)
                break;
        };

        return selectedAction;
    }
#endif

    StatePtr MCTSPolicy::executeAction(const StatePtr &state, const ActionPtr &action) {
        return env_->step(state, action);
    }

    void MCTSPolicy::backpropagate(NodePtr node, double reward) {
        while(node != nullptr)
        {
            node->visits += 1;
            node->value += reward;
            node = node->parent;
        }

    }

    double MCTSPolicy::simulate(const StatePtr &state, int maxSimSteps) {
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

    double MCTSPolicy::step() {
        do {
            root_ = search(maxIterations_);
            root_->isTerminal = env_->isTerminal(root_->state);
        }while(!root_->isTerminal);

        double episodeReward = env_->getTerminalReward(root_->state);
        backpropagate(root_, episodeReward);
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

    NodePtr MCTSPolicy::search(int max_iterations) {

        std::vector<NodePtr>children(max_iterations);
        std::vector<std::future<double>> parallel(max_iterations);

        for (int i = 0; i < max_iterations; ++i) {
            auto node = select(root_);
            auto child = expand(node);
            children[i] = child;
            parallel[i] = std::async(std::launch::async, &MCTSPolicy::simulate, this,
                                     std::ref(child->state), std::ref(maxSimSteps_));

        }

        NodePtr bestNode = nullptr;
        double bestval = -std::numeric_limits<double>::infinity();
        for (int i = 0; i < max_iterations; ++i) {
            auto reward = parallel[i].get();
            if(reward > bestval) {
                bestval = reward;
                bestNode = children[i];
            }
        }
        return bestNode;
    }

    NodePtr MCTSPolicy::getPolicy() const {
        return root_;
    }
} // mcts