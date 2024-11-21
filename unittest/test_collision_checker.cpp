//
// Created by airlab on 11/21/24.
//
#include <gtest/gtest.h>
#include "model/PremitiveCollisionChecker.h"
#include "model/EnhancedCollisionChecker.h"

TEST(CollisionCheckerTest, PremitiveCollisionChecker) {

    std::vector<std::vector<float>> obstacles{
        {5.,  5., 0.25},
        { 7.,  3., 0.25},
        { 3.,  7., 0.25}
    };
    double robot_radius = 0.25;
    env::PremitiveCollisionChecker cc(obstacles, robot_radius);

    std::vector<double> point1{5.0, 5.2};
    std::vector<double> point2{7.0, 7.2};

    ASSERT_TRUE(cc.is_collision(point1));
    ASSERT_FALSE(cc.is_collision(point2));

}

TEST(CollisionCheckerTest2, EnhancedCollisionChecker) {

    std::vector<std::vector<float>> obstacles{
            {5.,  5., 0.25},
            { 7.,  3., 0.25},
            { 3.,  7., 0.25}
    };
    double robot_radius = 0.25;
    env::EnhancedCollisionChecker cc(obstacles, robot_radius);

    std::vector<double> point1{5.0, 5.2};
    std::vector<double> point2{7.0, 7.2};

    ASSERT_TRUE(cc.is_collision(point1));
    ASSERT_FALSE(cc.is_collision(point2));

}