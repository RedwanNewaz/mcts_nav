//
// Created by airlab on 11/18/24.
//

#ifndef MCTS_STATE_H
#define MCTS_STATE_H
#include <iostream>
#include <vector>
#include <iterator>
#include <cmath>

namespace base{
    class State{
    public:
        State(const std::vector<double>& resolution): res_(resolution)
        {
            size_ = resolution.size();
            state_.resize(size_);
        }

        virtual ~State() {

        }

        std::vector<double> getResolution() const
        {
            return res_;
        }

        std::vector<double> getArray() const
        {
            return state_;
        }

        std::size_t operator()(const State& obj) const {
            return obj.hash();
        }

        friend std::ostream &operator<<(std::ostream &os, const State &state) {
            os << "state: ";
            std::copy(state.state_.begin(), state.state_.end(), std::ostream_iterator<double>(os, " "));
            return os;
        }

        std::size_t hash() const {
            std::size_t seed = 0;
            int i = 0;
            for (auto u : state_) {
                int a = static_cast<int>(u / res_[i++]);
                seed ^= std::hash<int>{}(a) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }

        bool operator == (const State& other) const
        {
            return hash() == other.hash();
        }

        double distance(const State& other) const
        {
            auto dx = state_[0] - other.state_[0];
            auto dy = state_[1] - other.state_[1];
            return std::hypot(dx, dy);
        }
    protected:
        std::size_t size_;
        std::vector<double> res_;
        std::vector<double> state_;
    };
}


// Specialization of std::hash for State
namespace std {
    template<>
    struct hash<base::State> {
        std::size_t operator()(const base::State& obj) const {
            return obj.hash();
        }
    };
}

using StatePtr = std::shared_ptr<base::State>;
#endif //MCTS_STATE_H
