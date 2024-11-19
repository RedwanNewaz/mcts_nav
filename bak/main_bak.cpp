#include "utility/ParamManager.h"
#include "env/CircularStaticObstacles.h"
#include "algo/MCTSPolicy.h"
#include <iostream>




int main(int argc, char* argv[]) {
    param_manager pm(argv[1]);
    std::vector<std::vector<float>> obstacles;
    pm.get_obstacles(obstacles);
    auto goal = pm.get_param<std::vector<float>>("goal");
    auto robotRadius = pm.get_param<double>("robot_radius");
    auto goal_radius = pm.get_param<double>("goal_radius");
    auto env1(std::make_shared<env::StaticObstaclesEnv>(goal, obstacles, robotRadius, goal_radius));
    mcts::MCTSPolicy policy(1, "", env1);
    policy.run();

    return 0;
}
