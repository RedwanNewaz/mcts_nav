#include "utility/ParamManager.h"
#include "env/CircularStaticObstacles.h"
#include "env/DynamicObstacles.h"
#include "algo/MCTSPolicy.h"
#include <queue>




int main(int argc, char* argv[]) {
    auto pm(std::make_shared<param_manager>(argv[1]));
    std::vector<std::vector<float>> obstacles;
    pm->get_obstacles(obstacles);
    auto start = pm->get_param<std::vector<float>>("start");
    auto goal = pm->get_param<std::vector<float>>("goal");
    auto robotRadius = pm->get_param<double>("robot_radius");
    auto goal_radius = pm->get_param<double>("goal_radius");
    auto dt = pm->get_param<double>("dt");

    auto max_speed = pm->get_param<double>("max_speed");
    auto min_speed = pm->get_param<double>("min_speed");
    auto max_yawrate = pm->get_param<double>("max_yawrate");

    auto v_reso = pm->get_param<double>("v_reso");
    auto yawrate_reso = pm->get_param<double>("yawrate_reso");

    auto train_epoch = pm->get_param<int>("train_epoch");
//    auto env1(std::make_shared<env::StaticObstaclesEnv>(start, goal, obstacles, robotRadius, goal_radius, dt));
    auto env1(std::make_shared<env::DynamicObstacles>(pm));


    std::vector<double> u_range{min_speed, max_speed, -max_yawrate, max_yawrate};
    std::vector<double> u_res{v_reso, yawrate_reso};

    mcts::MCTSPolicy policy(env1, u_range, u_res, train_epoch);
    policy.run();

    auto strategy = policy.getPolicy();
    std::queue<mcts::Node> queue;
    queue.push(*strategy);

    int depth = 0;
    env1->reset();
    while (!queue.empty()) {
       auto node = queue.front();
       queue.pop();

       std::cout << "[Depth ] " << ++depth << " | " << *node.state << " " << env1->isCollision(node.state) <<  std::endl;

       if(node.isTerminal)
           break;

       std::priority_queue<mcts::Node, std::vector<mcts::Node>, std::greater<mcts::Node>> bestChildren;
       for(auto& child:node.children)
           if(child->value > 0)
                bestChildren.push(*child);
       auto selectedChild = bestChildren.top();
       queue.push(selectedChild);

       env1->incrementTime();
    }

    return 0;
}
