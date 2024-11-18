//
// Created by airlab on 11/18/24.
//

#ifndef MCTS_DIFFWHEELROBOTSTATE_H
#define MCTS_DIFFWHEELROBOTSTATE_H
#include "base/state.h"

namespace model{
    class DiffWheelRobotState: public base::State
    {
    public:
        explicit DiffWheelRobotState(const std::vector<double> &state, const std::vector<double> &resolution) : State(resolution)
        {
            std::copy(state.begin(), state.end(), state_.begin());
        }
    };
}
#endif //MCTS_DIFFWHEELROBOTSTATE_H
