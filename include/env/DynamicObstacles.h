//
// Created by airlab on 11/21/24.
//

#ifndef MCTS_DYNAMICOBSTACLES_H
#define MCTS_DYNAMICOBSTACLES_H
#include <memory>
#include "base/env.h"
#include "model/DiffWheelRobotState.h"
#include "model/EnhancedCollisionChecker.h"
#include "ObstacleGenerator.h"
namespace env{
    class DynamicObstacles: public base::environment{
    public:
        DynamicObstacles(const ParamPtr& pm):pm_(pm)
        {
            auto start = pm->get_param<std::vector<float>>("start");
            auto goal = pm->get_param<std::vector<float>>("goal");
            robotRadius_ = pm->get_param<double>("robot_radius");
            goalRadius_ = pm->get_param<double>("goal_radius");
            sim_time_ = pm->get_param<double>("sim_time");
            obsLen_ = pm->get_param<double>("obstacle_length");
            dt_ = pm->get_param<double>("dt");
            pm->get_obstacles(obstacles_);
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

            // configure obs generator
            int num_timesteps = static_cast<int>(sim_time_ / dt_);
            obsGen_ = std::make_shared<ObstacleGenerator>(sim_time_, num_timesteps, pm);

            // configure collision checker
            dynObstacles_.resize(obstacles_.size(), std::vector<float>(3, 0));
            time_ = -dt_;
            incrementTime();
        }

        void incrementTime()
        {
            time_ += dt_;
            time_ = fmod(time_, sim_time_);

            auto positions = obsGen_->getObstaclePositionsAtTime(time_);
            // populate obstacle location
            for (int i = 0; i < positions.rows(); ++i) {
                for (int j = 0; j < positions.cols(); ++j) {
                    dynObstacles_[i][j] = positions(i, j);
                }
                dynObstacles_[i][2] = obsLen_;
            }
            collisionChecker_ = std::make_unique<env::EnhancedCollisionChecker>(dynObstacles_, robotRadius_);

        }

        bool isCollision(const StatePtr& state) const override
        {
            auto x = state->getArray();
            // avoid time index 0
            std::vector<double>X{x[1], x[2]};
            return collisionChecker_->is_collision(X);
        }

        std::vector<std::vector<float>> getDynObsList()const
        {
            return dynObstacles_;
        }


        StatePtr reset() override {
            time_ = -dt_;
            incrementTime();
            return startState_;
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


        bool isTerminal(const StatePtr& state) const override
        {
            double goalDist = state->distance(*goalState_);
            return (goalDist < goalRadius_) || isCollision(state);
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
        double goalRadius_;
        double robotRadius_;
        double obsLen_;
        double dt_;
        double time_;
        double sim_time_;
        ParamPtr pm_;

        StatePtr startState_;
        StatePtr currentState_;
        StatePtr goalState_;
        ObsGenPtr obsGen_;
        std::vector<std::vector<float>> dynObstacles_;
        std::vector<std::vector<float>> obstacles_;
        std::unique_ptr<env::EnhancedCollisionChecker> collisionChecker_;
    };
}
#endif //MCTS_DYNAMICOBSTACLES_H
