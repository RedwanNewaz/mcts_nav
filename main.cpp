#include "utility/ParamManager.h"
#include "env/DynamicObstacles.h"
#include "algo/MCTSPolicy.h"
#include "utility/simulator.h"

int main(int argc, char* argv[]) {

    auto pm(std::make_shared<param_manager>(argv[1]));
    std::vector<std::vector<float>> obstacles;
    pm->get_obstacles(obstacles);
    auto robotRadius = pm->get_param<double>("robot_radius");

    auto max_speed = pm->get_param<double>("max_speed");
    auto min_speed = pm->get_param<double>("min_speed");
    auto max_yawrate = pm->get_param<double>("max_yawrate");

    auto v_reso = pm->get_param<double>("v_reso");
    auto yawrate_reso = pm->get_param<double>("yawrate_reso");

    auto train_epoch = pm->get_param<int>("train_epoch");
    auto env1(std::make_shared<env::DynamicObstacles>(pm));


    std::vector<double> u_range{min_speed, max_speed, -max_yawrate, max_yawrate};
    std::vector<double> u_res{v_reso, yawrate_reso};

    std::string savePath = "../results/train001/policy.dat";
    mcts::MCTSPolicy policy(env1, u_range, u_res, train_epoch,savePath);
    policy.run();

    auto strategy = policy.getPolicy();
    Simulator::executePolicy(strategy, env1, robotRadius);


    return 0;
}
