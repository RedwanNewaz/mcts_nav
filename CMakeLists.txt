cmake_minimum_required(VERSION 3.22)
project(mcts)

set(CMAKE_CXX_STANDARD 17)
find_package(YAML-CPP REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(
        /usr/include/eigen3
        /opt/local/include/
        /usr/include/python3.10
        /usr/include/python3.10/numpy
        /Users/redwan/miniconda3//envs/limbo/include/python3.10
        /Users/redwan/miniconda3//envs/limbo/lib/python3.10/site-packages/numpy/core/include
        include)
set(SRC_FILES
        include/base/train.h
        include/base/action.h
        include/base/state.h
        include/model/UnicycleAction.h
        include/model/DiffWheelRobotState.h
        src/ParamManager.cpp
        src/MCTSPolicy.cpp
        include/algo/MCTSPolicy.h
        include/env/CircularStaticObstacles.h
        include/base/env.h
        include/base/collision_manager.h
        include/model/PremitiveCollisionChecker.h
        include/model/EnhancedCollisionChecker.h
        src/ObstacleGenerator.cpp
        include/env/DynamicObstacles.h
        include/utility/simulator.h
        include/algo/MCTSNode.h
)
link_directories(/Users/redwan/miniconda3/envs/limbo/lib/)

add_executable(mcts main.cpp ${SRC_FILES})
target_link_libraries(mcts
        yaml-cpp::yaml-cpp
        Eigen3::Eigen
        -lpython3.10
)


add_subdirectory(unittest)
