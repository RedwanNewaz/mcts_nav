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
            // state vec : t, x, y, theta, v, w
            std::vector<double> res(6, robotRadius_);
            res[0] = dt_;
            std::vector<double> state(6, 0.0);
            time_ = -dt_;
            incrementTime();

            state[0] = time_;
            state[1] = start[0];
            state[2] = start[1];
            state[3] = start[2];
            startState_ = std::make_shared<model::DiffWheelRobotState>(state, res);
            currentState_ = startState_;

            // configure obs generator
            int num_timesteps = static_cast<int>(sim_time_ / dt_);
            obsGen_ = std::make_shared<ObstacleGenerator>(sim_time_, num_timesteps, pm);
        }

        void incrementTime()
        {
            time_ += dt_;
            time_ = fmod(time_, sim_time_);
            std::vector<std::vector<float>> dynObstacles;
            auto positions = obsGen_->getObstaclePositionsAtTime(time_);
            // populate obstacle location
            for (int i = 0; i < positions.rows(); ++i) {
                std::vector<float>obsPos(3);
                for (int j = 0; j < positions.cols(); ++j) {
                    obsPos[j] = positions(i, j);
                }
                obsPos[2] = obsLen_;
                dynObstacles.push_back(obsPos);
            }
            collisionChecker_ = std::make_unique<env::EnhancedCollisionChecker>(dynObstacles, robotRadius_);

        }

        bool isCollision(const StatePtr& state) const override
        {
            auto x = state->getArray();
            // avoid time index 0
            std::vector<double>X{x[1], x[2]};
            return collisionChecker_->is_collision(X);
        }


        StatePtr reset() override {
            time_ = -dt_;
            incrementTime();
            return startState_;
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
        ObsGenPtr obsGen_;
        std::vector<std::vector<float>> obstacles_;
        std::unique_ptr<env::EnhancedCollisionChecker> collisionChecker_;
    };
}
#endif //MCTS_DYNAMICOBSTACLES_H
