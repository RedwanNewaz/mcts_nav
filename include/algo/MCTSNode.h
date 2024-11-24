//
// Created by redwan on 11/24/24.
//

#ifndef MCTS_MCTSNODE_H
#define MCTS_MCTSNODE_H
#include <memory>
#include "base/state.h"
#include "base/action.h"
#include "base/train.h"
#include "base/env.h"

namespace mcts{
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

        // Serialization function
        void serialize(std::ofstream& out) const {
            // Serialize state and action (assuming they have serialization methods)
            state->serialize(out);
            action->serialize(out);

            // Serialize children
            size_t childCount = children.size();
            out.write(reinterpret_cast<const char*>(&childCount), sizeof(childCount));
            for (const auto& child : children) {
                child->serialize(out);
            }

            // Serialize primitive types
            out.write(reinterpret_cast<const char*>(&visits), sizeof(visits));
            out.write(reinterpret_cast<const char*>(&value), sizeof(value));
            out.write(reinterpret_cast<const char*>(&isTerminal), sizeof(isTerminal));
        }

    };
    using NodePtr = std::shared_ptr<Node>;
}
#endif //MCTS_MCTSNODE_H
