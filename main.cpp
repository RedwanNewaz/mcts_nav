#include "utility/ParamManager.h"
#include "env/CircularStaticObstacles.h"
#include "algo/MCTSPolicy.h"





int main(int argc, char* argv[]) {
    param_manager pm(argv[1]);
    std::vector<std::vector<float>> obstacles;
    pm.get_obstacles(obstacles);
    auto start = pm.get_param<std::vector<double>>("start");
    auto goal = pm.get_param<std::vector<float>>("goal");
    auto robotRadius = pm.get_param<double>("robot_radius");
    auto goal_radius = pm.get_param<double>("goal_radius");
    auto train_epoch = pm.get_param<int>("train_epoch");
    auto env1(std::make_shared<env::StaticObstaclesEnv>(goal, obstacles, robotRadius, goal_radius));

    std::vector<double> reso(5, robotRadius);
    std::vector<double> x(5, 0.0);
    x[0] = start[0];
    x[1] = start[1];
    x[2] = start[2];

    std::vector<double> u_range{0, 1, -0.5, 0.5};
    std::vector<double> u_res{0.1, 0.1};

    auto initX(std::make_shared<model::DiffWheelRobotState>(x, reso));

    mcts::MCTSPolicy policy(initX, env1, u_range, u_res, train_epoch);
    policy.run();

    return 0;
}
