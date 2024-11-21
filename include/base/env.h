//
// Created by airlab on 11/18/24.
//

#ifndef MCTS_ENV_H
#define MCTS_ENV_H
#include <memory>
#include "base/state.h"
#include "base/action.h"
namespace base{
    class environment{
    public:
        virtual ~environment() = default;

        virtual double getReward(const StatePtr& state) const = 0;
        virtual bool isCollision(const StatePtr& state) const = 0;
        virtual bool isTerminal(const StatePtr& state) const = 0;
        virtual StatePtr step(const StatePtr &state, const ActionPtr &action) = 0;
        virtual StatePtr reset() = 0;
        virtual double getTerminalReward(const StatePtr &state)
        {
            return 0;
        }
    };
}
using EnvPtr = std::shared_ptr<base::environment>;
#endif //MCTS_ENV_H
