//
// Created by redwan on 11/10/24.
//

#include "utility/ParamManager.h"

bool Matrix2D::operator==(const Matrix2D& other) const {
    return other.data.size() == data.size() && other.data[0].size() == data[0].size();
}

std::ostream& operator<<(std::ostream& os, const Matrix2D& d) {
    for (const auto& row : d.data) {
        for (float value : row) {
            os << value << " ";
        }
        os << "\n";
    }
    return os;
}

YAML::Node YAML::convert<Matrix2D>::encode(const Matrix2D& rhs) {
    Node node;
    for (const auto& d : rhs.data) {
        node.push_back(d);
    }
    return node;
}

bool YAML::convert<Matrix2D>::decode(const Node& node, Matrix2D& rhs) {
    for (const auto& temp : node) {
        std::vector<float> cols;
        for (const auto& item : temp) {
            cols.push_back(item.as<float>());
        }
        rhs.data.push_back(cols);
    }
    return true;
}

param_manager::param_manager(const std::string& file) {
    config_ = YAML::LoadFile(file);
}

std::shared_ptr<param_manager> param_manager::getSharedPtr() {
    return shared_from_this();
}
