//
// Created by redwan on 11/10/24.
//

#ifndef BOW_PARAMMANAGER_H
#define BOW_PARAMMANAGER_H



#include <yaml-cpp/yaml.h>
#include <memory>
#include <vector>
#include <string>

struct Matrix2D {
    std::vector<std::vector<float>> data;
    bool operator==(const Matrix2D& other) const;
    friend std::ostream& operator<<(std::ostream& os, const Matrix2D& d);
};

namespace YAML {
    template<>
    struct convert<Matrix2D> {
        static Node encode(const Matrix2D& rhs);
        static bool decode(const Node& node, Matrix2D& rhs);
    };
}

class param_manager : public std::enable_shared_from_this<param_manager> {
public:
    explicit param_manager(const std::string& file);

    template<class T>
    T get_param(const std::string& field);

    template<class T>
    T get_param(const std::string& field1, const std::string& field2);

    template<class T>
    T get_param(const std::string& field1, const std::string& field2, const std::string& field3);

    template<class T>
    void get_obstacles(T& result);

    std::shared_ptr<param_manager> getSharedPtr();

private:
    YAML::Node config_;
};

using ParamPtr = std::shared_ptr<param_manager>;

#include "param_manager.tpp"



#endif //BOW_PARAMMANAGER_H
