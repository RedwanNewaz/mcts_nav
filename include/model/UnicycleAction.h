//
// Created by airlab on 11/18/24.
//

#ifndef MCTS_UNICYCLEACTION_H
#define MCTS_UNICYCLEACTION_H
#include "base/action.h"
#include <iomanip>
#include <algorithm>
#include <random>
#include <numeric>

namespace model{
class UnicycleAction: public base::Action
{
public:
    /**
     * Unicycle motion model
     * @param range  vector of (v_res, w_res)
     * @param resolution (v_min, v_max, w_min, w_max)
     * @param v linear velocity
     * @param w angular velocity
     */
    UnicycleAction(const std::vector<double>& range, const std::vector<double>& resolution, double v=0, double w=0):
            base::Action(resolution), range_(range)
    {
        act_[0] = v;
        act_[1] = w;
    }
    std::vector<UnicycleAction> getAllPossibleActions() const
    {
        std::vector<UnicycleAction> ACTIONS;
        auto linear = createRangeVector(range_[0], range_[1], res_[0]);
        auto angular = createRangeVector(range_[2], range_[3], res_[1]);

        for(auto& v: linear)
            for(auto& w: angular)
                ACTIONS.emplace_back(UnicycleAction(range_, res_, v, w));
        return ACTIONS;
    }
    std::size_t allPossibleActionSize() const
    {
        int v_size = static_cast<int>((range_[1] - range_[0]) / res_[0]) + 1;
        int w_size = static_cast<int>((range_[3] - range_[2]) / res_[1]) + 1;
        return v_size * w_size;
    }

    template<class T>
    std::vector<T> sampleRandomActions(const std::vector<T>& ACTIONS, int sample_size) const
    {
        std::vector<T> out;
        std::sample(ACTIONS.begin(), ACTIONS.end(), std::back_inserter(out), sample_size,
                    std::mt19937 {std::random_device{}()});
        return out;
    }
private:
    std::vector<double> range_;

protected:
    std::vector<double> createRangeVector(double min_value, double max_value, double resolution) const {
        int size = static_cast<int>((max_value - min_value) / resolution) + 1;
        std::vector<double> range_vec(size);

        std::iota(range_vec.begin(), range_vec.end(), 0);
        std::transform(range_vec.begin(), range_vec.end(), range_vec.begin(),
                       [min_value, resolution](double i) { return min_value + i * resolution; });

        return range_vec;
    }


};
}

#endif //MCTS_UNICYCLEACTION_H
