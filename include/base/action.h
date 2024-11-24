//
// Created by airlab on 11/18/24.
//

#ifndef MCTS_ACTION_H
#define MCTS_ACTION_H
#include <iostream>
#include <vector>
#include <iterator>
#include <memory>

namespace base{
class Action{
    public:
        Action(const std::vector<double>& resolution): res_(resolution)
        {
            size_ = resolution.size();
            act_.resize(size_);
        }



    virtual ~Action() = default;
        std::size_t operator()(const Action& obj) const {
            return obj.hash();
        }

        friend std::ostream &operator<<(std::ostream &os, const Action &action) {
            os << "action: ";
            std::copy(action.act_.begin(), action.act_.end(), std::ostream_iterator<double>(os, " "));
            return os;
        }

        std::vector<double> getArray() const
        {
            return act_;
        }

        bool operator != (const Action& other) const
        {
            return hash() != other.hash();
        }

        bool operator == (const Action& other) const
        {
            return hash() == other.hash();
        }

        [[nodiscard]] std::size_t hash() const {
            std::size_t seed = 0;
            int i = 0;
            for (auto u : act_) {
                int a = static_cast<int>(u / res_[i++]);
                seed ^= std::hash<int>{}(a) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }

    void serialize(std::ofstream& out) {
        // Write the size of the vector
        size_t size = act_.size();
        out.write(reinterpret_cast<const char*>(&size), sizeof(size));
        // Write the vector elements
        out.write(reinterpret_cast<const char*>(act_.data()), act_.size() * sizeof(double));
        out.write(reinterpret_cast<const char*>(res_.data()), res_.size() * sizeof(double));

    }

    static std::vector<double> deserialize(std::ifstream& in) {
        // Read the size of the vector
        size_t size;
        in.read(reinterpret_cast<char*>(&size), sizeof(size));
        // Create and read the vector elements
        // read for data and resolution
        std::vector<double> vec(2 * size);
        in.read(reinterpret_cast<char*>(vec.data()), size * sizeof(double));

        return vec;
    }
    protected:
        std::size_t size_;
        std::vector<double> res_;
        std::vector<double> act_;
    };
}


// Specialization of std::hash for ACTION
namespace std {
    template<>
    struct hash<base::Action> {
        std::size_t operator()(const base::Action& obj) const {
            return obj.hash();
        }
    };
}



using ActionPtr = std::shared_ptr<base::Action>;
#endif //MCTS_ACTION_H
