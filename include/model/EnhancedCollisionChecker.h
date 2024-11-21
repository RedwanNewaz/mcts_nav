//
// Created by airlab on 11/21/24.
//

#ifndef ENHANCEDCOLLISIONCHECKER_H
#define ENHANCEDCOLLISIONCHECKER_H
#include <cmath>
#include <unordered_set>
#include "base/collision_manager.h"
namespace env{
    // Alias for the point type
    using POINT = std::pair<int, int>;

    // Custom hash function for POINT
    struct PointHash {
        std::size_t operator()(const POINT& p) const {
            return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
        }
    };

    // Custom equality comparator for POINT (optional, default works for pairs)
    struct PointEqual {
        bool operator()(const POINT& p1, const POINT& p2) const {
            return p1.first == p2.first && p1.second == p2.second;
        }
    };

    class EnhancedCollisionChecker : public CollisionManager{
    public:

        EnhancedCollisionChecker(const std::vector<std::vector<float>> &obstacles, double robotRadius)
            : CollisionManager(obstacles, robotRadius) {
            for (auto &obs: obstacles_) {
                double maxr = robotRadius_ + obs.radius;
                for (double theta = 0; theta < 2 * M_PI; theta += M_PI / 16) {
                    for (double r = 0; r < maxr; r += robotRadius_) {

                        double xx = obs.x + r * cos(theta);
                        double yy = obs.y + r * sin(theta);

                        int X = static_cast<int>(xx / robotRadius_);
                        int Y = static_cast<int>(yy / robotRadius_);
                        obstacles_set_.insert({X, Y});
                    }
                }
            }

            //printf("obstacles_set_.size(): %zu\n", obstacles_set_.size());
        }

    protected:
        bool check_collision(double x, double y) override {
            int X = static_cast<int>( x / robotRadius_);
            int Y = static_cast<int> (y / robotRadius_);

            if (obstacles_set_.find({X, Y}) != obstacles_set_.end()) {
                return true;
            }
            return false;
        }

    private:
        std::unordered_set<POINT, PointHash, PointEqual> obstacles_set_;

    };
}

#endif //ENHANCEDCOLLISIONCHECKER_H
