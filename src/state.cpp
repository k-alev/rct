#include <iostream>
#include <state.h>

namespace rct
{
State::State() 
{
    std::cout << "State Constructed!!!" << std::endl;
}

State::State(Eigen::Matrix<double, 6, 1> x, Eigen::Matrix<double, 6, 1> dx, Eigen::Matrix<double, 6, 1> ddx, Eigen::Matrix<double, 6, 1> ft, Eigen::Matrix<double, 3, 3> rot)
    : pos{x}, vel{dx}, acc{ddx}, wrench{ft}, R{rot}
{
    std::cout << "State REConstructed!!!" << std::endl;
}

} // namespace rct