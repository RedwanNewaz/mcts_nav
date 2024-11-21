//
// Created by redwan on 11/10/24.
//

#ifndef BOW_OBSTACLEGENERATOR_H
#define BOW_OBSTACLEGENERATOR_H
#include <vector>
#include <Eigen/Dense>
#include <memory>
#include "utility/ParamManager.h"

namespace env {
    struct Obstacle {
        Eigen::Vector2d initial_position;
        double velocity;
        double angle;
    };

    class ObstacleGenerator {
    public:
        using TWD_VEC = std::vector<std::vector<float>>;
        ObstacleGenerator(double sim_time, int num_timesteps, const ParamPtr& pm);
        Eigen::MatrixXd getObstaclePositionsAtTime(double t) const;

    private:
        double sim_time_;
        int num_timesteps_;
        double obstacle_length_;
        double robot_radius_;
        ParamPtr pm_;
        std::vector<Obstacle> dyn_obs_param_;
        TWD_VEC obsList_;

        Eigen::MatrixXd createRobot(const Eigen::Vector2d& p0, double v, double theta);
        Eigen::MatrixXd cumulativeSum(const Eigen::MatrixXd& matrix, int dim);
        // New helper method
        Eigen::Vector2d calculatePosition(const Obstacle& obstacle, double t) const;
        Eigen::MatrixXd createObstacles(const std::vector<Obstacle>& obstacle_params);
        Eigen::MatrixXd getRobotPositionsAtTime(const std::vector<Obstacle>& obstacle_params, double t) const;
    };

} // env
using ObsGenPtr = std::shared_ptr<env::ObstacleGenerator>;
#endif //BOW_OBSTACLEGENERATOR_H