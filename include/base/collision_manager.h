//
// Created by airlab on 11/21/24.
//

#ifndef COLLISION_MANAGER_H
#define COLLISION_MANAGER_H
namespace env {
    class CollisionManager {
    public:
        virtual ~CollisionManager() = default;

        struct CircularObstacle{
            double x, y, radius;
        };
        CollisionManager(const std::vector<std::vector<float>>& obstacles, double robotRadius):
        robotRadius_(robotRadius)
        {
            for(auto& o:obstacles)
            {
                CircularObstacle c{o[0], o[1], o[2]};
                obstacles_.emplace_back(c);
            }
        }
        template <typename T>
        bool is_collision(const T& state) {
            return check_collision(state[0], state[1]);
        }
    protected:
        std::vector<CircularObstacle> obstacles_;
        double robotRadius_;
        virtual bool check_collision(double x, double y) = 0;

    };
}

#endif //COLLISION_MANAGER_H
