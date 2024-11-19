//
// Created by airlab on 11/18/24.
//

#ifndef MCTS_CIRCULARSTATICOBSTACLES_H
#define MCTS_CIRCULARSTATICOBSTACLES_H
#include <memory>
#include "base/env.h"
#include "model/DiffWheelRobotState.h"

namespace env{
    struct CircularObstacle{
        double x, y, radius;
    };
    class StaticObstaclesEnv: public base::environment{
    public:
        StaticObstaclesEnv(const std::vector<float>& goal, const std::vector<std::vector<float>>& obstacles, double robotRadius, double goalRadius)
        :robotRadius_(robotRadius), goalRadius_(goalRadius)
        {
            for(auto& o:obstacles)
            {
                CircularObstacle c{o[0], o[1], o[2]};
                obstacles_.emplace_back(c);
            }
            std::vector<double> res(5, robotRadius_);
            std::vector<double> state(5, 0.0);
            state[0] = goal[0];
            state[1] = goal[1];
            goalState_ = std::make_shared<model::DiffWheelRobotState>(state, res);
        }
        double getReward(const StatePtr& state) const override
        {
            if(isTerminal(state))
                return 1.e3;
            if(isCollision(state))
                return -1.0e3;
            double goalDist = state->distance(*goalState_);
//            return 1.0e3 * exp(-goalDist);
            return -goalDist;
        }

        bool isCollision(const StatePtr& state) const override
        {
            auto x = state->getArray();
            for(auto& obstacle: obstacles_)
            {
                double dx = obstacle.x - x[0];
                double dy = obstacle.y - x[1];
                double obsDist = std::hypot(dx, dy);
                if(obsDist <= (obstacle.radius + robotRadius_))
                    return true;
            }
            return false;
        }

        bool isTerminal(const StatePtr& state) const override
        {
            double goalDist = state->distance(*goalState_);
            return goalDist < goalRadius_;
        }

        double goalDistance(const StatePtr& state) const
        {
            return state->distance(*goalState_);
        }

    private:
        double goalRadius_;
        double robotRadius_;
        StatePtr goalState_;
        std::vector<CircularObstacle> obstacles_;
    };
}
#endif //MCTS_CIRCULARSTATICOBSTACLES_H
