#ifndef PARAM_MANAGER_TPP
#define PARAM_MANAGER_TPP

template<class T>
T param_manager::get_param(const std::string& field) {
    return config_[field].template as<T>();
}

template<class T>
T param_manager::get_param(const std::string& field1, const std::string& field2) {
    return config_[field1][field2].template as<T>();
}

template<class T>
T param_manager::get_param(const std::string& field1, const std::string& field2, const std::string& field3) {
    return config_[field1][field2][field3].template as<T>();
}

template<class T>
void param_manager::get_obstacles(T& result) {
    Matrix2D obstacles = config_["obstacles"].as<Matrix2D>();
    std::copy(obstacles.data.begin(), obstacles.data.end(), std::back_inserter(result));
}

#endif // PARAM_MANAGER_TPP