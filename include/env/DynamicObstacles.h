//
// Created by airlab on 11/21/24.
//

#ifndef MCTS_DYNAMICOBSTACLES_H
#define MCTS_DYNAMICOBSTACLES_H
#include <memory>
#include "CircularStaticObstacles.h"
#include "model/DiffWheelRobotState.h"
#include "model/EnhancedCollisionChecker.h"
#include "model/PremitiveCollisionChecker.h"
#include "ObstacleGenerator.h"
namespace env{
    class DynamicObstacles: public StaticObstaclesEnv{
    public:
        explicit DynamicObstacles(const ParamPtr& pm): StaticObstaclesEnv(pm)
        {
            sim_time_ = pm->get_param<double>("sim_time");
            obsLen_ = pm->get_param<double>("obstacle_length");
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
            // printf("obs size (%d, %d) \n", positions.rows(), positions.cols());
            // populate obstacle location
            for (int i = 0; i < positions.rows(); ++i) {
                for (int j = 0; j < positions.cols(); ++j) {
                    dynObstacles_[j][i] = positions(i, j);
                }
                dynObstacles_[i][2] = obsLen_;
            }
            collisionChecker_ = std::make_unique<env::EnhancedCollisionChecker>(dynObstacles_, robotRadius_);

        }


        double getRobotRadius() const {
            return robotRadius_;
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


    private:
        double obsLen_;
        double time_;
        double sim_time_;
        ObsGenPtr obsGen_;
        std::vector<std::vector<float>> dynObstacles_;
    };
}
#endif //MCTS_DYNAMICOBSTACLES_H
