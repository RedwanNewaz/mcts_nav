//
// Created by airlab on 11/21/24.
//

#ifndef PREMITIVECOLLISIONCHECKER_H
#define PREMITIVECOLLISIONCHECKER_H
#include <cmath>
#include "base/collision_manager.h"

namespace env {
    class PremitiveCollisionChecker : public CollisionManager{
    public:
        PremitiveCollisionChecker(const std::vector<std::vector<float>> &obstacles, double robotRadius)
            : CollisionManager(obstacles, robotRadius) {
        }

    protected:
        bool check_collision(double x, double y) override {

            for(auto& obstacle: obstacles_)
            {
                double dx = obstacle.x - x;
                double dy = obstacle.y - y;
                double obsDist = std::hypot(dx, dy);
                if(obsDist <= (obstacle.radius + robotRadius_))
                    return true;
            }
            return false;
        }
    };
}

#endif //PREMITIVECOLLISIONCHECKER_H
