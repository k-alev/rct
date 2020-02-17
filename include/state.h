#pragma once

#include <Eigen/Dense>

namespace rct
{
class State 
{
public:
    State();
    State(Eigen::Matrix<double, 6, 1> x, Eigen::Matrix<double, 6, 1> dx, Eigen::Matrix<double, 6, 1> ddx, Eigen::Matrix<double, 6, 1> ft, Eigen::Matrix<double, 3, 3> rot);
    Eigen::Matrix<double, 6, 1> pos;
    Eigen::Matrix<double, 6, 1> vel;
    Eigen::Matrix<double, 6, 1> acc;
    Eigen::Matrix<double, 6, 1> wrench;
    Eigen::Matrix<double, 3, 3> R;

// private:


};

} // namespace rct