//
// Created by redwan on 11/22/24.
//

#ifndef SIMULATOR_H
#define SIMULATOR_H
#include "utility/matplotlibcpp.h"
#include "env/DynamicObstacles.h"
#include "algo/MCTSPolicy.h"

namespace plt = matplotlibcpp;
typedef std::vector<std::vector<float>> OBS_TYPE;
typedef std::shared_ptr<env::DynamicObstacles> DynEnvPtr;

struct Simulator {

    template <typename T>
    static void executePolicy(const T& strategy, const DynEnvPtr& env1, double robotRadius) {
        std::queue<mcts::Node> queue;
        queue.push(*strategy);

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
            auto selectedChild = bestChildren.top();
            queue.push(selectedChild);

            env1->incrementTime();
        }
    }

    static void animate(const StatePtr& state, const OBS_TYPE& obs, double robotRadius) {

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
        plt::grid(true);
        plt::xlim(-5, 20);
        plt::ylim(-5, 20);
        plt::pause(0.01);
    }
};

#endif //SIMULATOR_H
