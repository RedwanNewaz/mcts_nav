//
// Created by airlab on 11/18/24.
//

#ifndef MCTS_CIRCULARSTATICOBSTACLES_H
#define MCTS_CIRCULARSTATICOBSTACLES_H
#include <memory>
#include "base/env.h"
#include "model/DiffWheelRobotState.h"
#include "model/EnhancedCollisionChecker.h"

namespace env{

    class StaticObstaclesEnv: public base::environment{
    public:
        StaticObstaclesEnv(const std::vector<float>& start, const std::vector<float>& goal, const std::vector<std::vector<float>>& obstacles,
                           double robotRadius, double goalRadius, double dt)
        :robotRadius_(robotRadius), goalRadius_(goalRadius), dt_(dt)
        {


            collisionChecker_ = std::make_unique<env::EnhancedCollisionChecker>(obstacles, robotRadius);
            // state vec : x, y, theta, v, w
            std::vector<double> res(5, robotRadius_);
            std::vector<double> state(5, 0.0);
            state[0] = goal[0];
            state[1] = goal[1];
            goalState_ = std::make_shared<model::DiffWheelRobotState>(state, res);

            state[0] = start[0];
            state[1] = start[1];
            state[2] = start[2];
            startState_ = std::make_shared<model::DiffWheelRobotState>(state, res);
            currentState_ = std::make_shared<model::DiffWheelRobotState>(state, res);
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

        double getTerminalReward(const StatePtr &state) override {
            double goalDist = state->distance(*goalState_);

            return (goalDist < goalRadius_) ? 1.0 : -1.0;
        }

        bool isCollision(const StatePtr& state) const override
        {
           return collisionChecker_->is_collision(state->getArray());
        }

        bool isTerminal(const StatePtr& state) const override
        {
            double goalDist = state->distance(*goalState_);
            return (goalDist < goalRadius_) || isCollision(state);
        }

        StatePtr reset() override {
            return startState_;
        }

        void setState(const StatePtr& state)
        {
            currentState_ = state;
        }

        StatePtr step(const StatePtr &state, const ActionPtr &action) override {

            auto s = state->getArray();
            auto u = action->getArray();
            double theta = s[2] + u[1] * dt_;
            double x = s[0] + u[0] * cos(theta) * dt_;
            double y = s[1] + u[0] * sin(theta) * dt_;
            std::vector<double> X{x, y, theta, u[0], u[1]};
            return std::make_shared<model::DiffWheelRobotState>(X, state->getResolution());
        }


    private:
        const double goalRadius_;
        const double robotRadius_;
        const double dt_;

        StatePtr goalState_;
        StatePtr startState_;
        StatePtr currentState_;
        std::unique_ptr<env::EnhancedCollisionChecker> collisionChecker_;

    };
}
#endif //MCTS_CIRCULARSTATICOBSTACLES_H
