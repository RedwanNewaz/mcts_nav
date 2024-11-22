#include "utility/ParamManager.h"
#include "env/CircularStaticObstacles.h"
#include "env/DynamicObstacles.h"
#include "algo/MCTSPolicy.h"
#include <queue>

#include "build/_deps/googletest-src/googletest/include/gtest/internal/gtest-port.h"
#include "utility/matplotlibcpp.h"

namespace plt = matplotlibcpp;
typedef std::vector<std::vector<float>> OBS_TYPE;

void animate(const StatePtr& state, const OBS_TYPE& obs, double robotRadius) {

    plt::clf();
    std::map<std::string, std::string> args;
    // show dynamic obstacles
    args["color"] = "black";
    auto cc(std::make_shared<env::EnhancedCollisionChecker>(obs, robotRadius));
    if(cc->is_collision(state->getArray())) {
        std::cout << "[collision ] " <<" | " << *state  <<  std::endl;

    }


    std::vector<float>X(obs.size()), Y(obs.size());
    int i = 0;
    for(auto& ob : obs) {
        X[i] = ob[0];
        Y[i] = ob[1];
        ++i;
      //  printf("obs size =  %d %lf %lf \n", i, X[i], Y[i]);
    }


    plt::scatter(X, Y, 50, args);

    auto robot = state->getArray();
    args["color"] = "blue";
    std::vector<double>rX{robot[0]}, rY{robot[1]};
    plt::scatter(rX, rY, 50, args);

    plt::xlim(-5, 20);
    plt::ylim(-5, 20);
    plt::pause(0.01);
}

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

       auto obstacles = env1->getDynObsList();
       animate(node.state, obstacles, robotRadius);

       // std::cout << "[Depth ] " << ++depth << " | " << *node.state << " " << env1->isCollision(node.state) <<  std::endl;

       if(node.isTerminal)
           break;

       std::priority_queue<mcts::Node, std::vector<mcts::Node>, std::greater<mcts::Node>> bestChildren;
       for(auto& child:node.children)
           bestChildren.push(*child);
           // if(child->value > 0)
           //      bestChildren.push(*child);
       auto selectedChild = bestChildren.top();
       queue.push(selectedChild);

       env1->incrementTime();
    }

    return 0;
}
