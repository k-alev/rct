#pragma once

#include <Eigen/Dense>

namespace rct
{
class Frame 
{
public:
    Frame()
    {
        std::cout << "Frame Constructed!!!" << std::endl;
    };
    Frame(Eigen::Matrix<double, 6, 1> x, Eigen::Matrix<double, 6, 1> dx, Eigen::Matrix<double, 6, 1> ddx, Eigen::Matrix<double, 6, 1> ft)
    : pos{x}, vel{dx}, acc{ddx}, wrench{ft}
    {
        std::cout << "Frame REConstructed!!!" << std::endl;
    };
    Eigen::Matrix<double, 6, 1> pos;
    Eigen::Matrix<double, 6, 1> vel;
    Eigen::Matrix<double, 6, 1> acc;
    Eigen::Matrix<double, 6, 1> wrench;

// private:

};


class Status 
{
public:
    Status()
    {
        std::cout << "Status Constructed!!!" << std::endl;
    };
    
    Eigen::MatrixXd J;
    Eigen::Matrix<double,6,1> dJdq;
    Eigen::Matrix<double,3,3> R;
    Eigen::MatrixXd q_conf;
    Eigen::MatrixXd dq_conf;
    Eigen::Quaterniond quat = Eigen::Quaterniond(0,0,0,1);  
    // Eigen::Matrix<double, 6, 1> w_l_base;
    // Eigen::MatrixXd jacInv;

    // Other
    Frame frame = Frame();
    Frame frame_ee = Frame();


};


} // namespace rct