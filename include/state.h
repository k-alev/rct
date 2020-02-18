#pragma once

#include <Eigen/Dense>

namespace rct
{
class State 
{
public:
    State()
    {
        std::cout << "State Constructed!!!" << std::endl;
    };
    State(Eigen::Matrix<double, 6, 1> x, Eigen::Matrix<double, 6, 1> dx, Eigen::Matrix<double, 6, 1> ddx, Eigen::Matrix<double, 6, 1> ft)
    : pos{x}, vel{dx}, acc{ddx}, wrench{ft}
    {
        std::cout << "State REConstructed!!!" << std::endl;
    };
    Eigen::Matrix<double, 6, 1> pos;
    Eigen::Matrix<double, 6, 1> vel;
    Eigen::Matrix<double, 6, 1> acc;
    Eigen::Matrix<double, 6, 1> wrench;

// private:


};

} // namespace rct