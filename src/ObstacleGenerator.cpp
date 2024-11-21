//
// Created by redwan on 11/10/24.
//

#include "env/ObstacleGenerator.h"
#include <cmath>

namespace env {

    ObstacleGenerator::ObstacleGenerator(double sim_time, int num_timesteps, const ParamPtr& pm)
            : sim_time_(sim_time), num_timesteps_(num_timesteps), pm_(pm) {
        obstacle_length_    = pm_->get_param<double>("obstacle_length");
        robot_radius_   = pm_->get_param<double>("robot_radius");

        std::vector<std::vector<float>> dynObsList;
        pm->get_obstacles(dynObsList);
        for(auto& dyobs: dynObsList)
        {
            Obstacle o{{dyobs[0], dyobs[1]}, dyobs[2], dyobs[3]};
            dyn_obs_param_.emplace_back(o);
        }
        obsList_.resize(dyn_obs_param_.size(), std::vector<float>(2, 0));
        createObstacles(dyn_obs_param_);

    }

    Eigen::MatrixXd ObstacleGenerator::createObstacles(const std::vector<Obstacle>& obstacle_params) {
        Eigen::MatrixXd obstacles(4, num_timesteps_ * obstacle_params.size());

        for (size_t i = 0; i < obstacle_params.size(); ++i) {
            Eigen::MatrixXd obst = createRobot(obstacle_params[i].initial_position,
                                               obstacle_params[i].velocity,
                                               obstacle_params[i].angle);
            obstacles.block(0, i * num_timesteps_, 4, num_timesteps_) = obst;
        }

        return obstacles;
    }

    Eigen::MatrixXd ObstacleGenerator::createRobot(const Eigen::Vector2d& p0, double v, double theta) {
        Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(num_timesteps_, 0, sim_time_);
        Eigen::VectorXd vx = v * Eigen::VectorXd::Constant(num_timesteps_, std::cos(theta));
        Eigen::VectorXd vy = v * Eigen::VectorXd::Constant(num_timesteps_, std::sin(theta));

        Eigen::MatrixXd velocity(2, num_timesteps_);
        velocity.row(0) = vx;
        velocity.row(1) = vy;

        Eigen::MatrixXd position = p0.replicate(1, num_timesteps_) +
                                   cumulativeSum(velocity, 1) * (sim_time_ / num_timesteps_);

        Eigen::MatrixXd result(4, num_timesteps_);
        result.topRows(2) = position;
        result.bottomRows(2) = velocity;

        return result;
    }

    Eigen::MatrixXd ObstacleGenerator::cumulativeSum(const Eigen::MatrixXd& matrix, int dim) {
        Eigen::MatrixXd result = matrix;
        if (dim == 0) {
            for (int col = 0; col < matrix.cols(); ++col) {
                for (int row = 1; row < matrix.rows(); ++row) {
                    result(row, col) += result(row - 1, col);
                }
            }
        } else {
            for (int row = 0; row < matrix.rows(); ++row) {
                for (int col = 1; col < matrix.cols(); ++col) {
                    result(row, col) += result(row, col - 1);
                }
            }
        }
        return result;
    }

    Eigen::MatrixXd ObstacleGenerator::getRobotPositionsAtTime(const std::vector<Obstacle>& obstacle_params, double t) const {
        if (t < 0 || t > sim_time_) {
            throw std::runtime_error("Time t is out of simulation range");
        }

        Eigen::MatrixXd positions(2, obstacle_params.size());

        for (size_t i = 0; i < obstacle_params.size(); ++i) {
            positions.col(i) = calculatePosition(obstacle_params[i], t);
        }

        return positions;
    }

    Eigen::Vector2d ObstacleGenerator::calculatePosition(const Obstacle& obstacle, double t) const {
        double dx = obstacle.velocity * std::cos(obstacle.angle) * t;
        double dy = obstacle.velocity * std::sin(obstacle.angle) * t;
        return obstacle.initial_position + Eigen::Vector2d(dx, dy);
    }

    Eigen::MatrixXd ObstacleGenerator::getObstaclePositionsAtTime(double t) const {
        return getRobotPositionsAtTime(dyn_obs_param_, t);
    }


} // mbow