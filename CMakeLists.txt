cmake_minimum_required(VERSION 3.22)
project(mcts)

set(CMAKE_CXX_STANDARD 17)
find_package(YAML-CPP REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(
        /usr/include/eigen3
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
)

add_executable(mcts main.cpp ${SRC_FILES})
target_link_libraries(mcts
        yaml-cpp::yaml-cpp
        Eigen3::Eigen
)


add_subdirectory(unittest)
