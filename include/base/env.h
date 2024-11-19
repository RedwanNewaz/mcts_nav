//
// Created by airlab on 11/18/24.
//

#ifndef MCTS_ENV_H
#define MCTS_ENV_H
#include <memory>
#include "base/state.h"
namespace base{
    class environment{
    public:
        virtual double getReward(const StatePtr& state) const = 0;
        virtual bool isCollision(const StatePtr& state) const = 0;
        virtual bool isTerminal(const StatePtr& state) const = 0;
        virtual double goalDistance(const StatePtr& state) const = 0;
    };
}
using EnvPtr = std::shared_ptr<base::environment>;
#endif //MCTS_ENV_H
