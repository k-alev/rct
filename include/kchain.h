#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <kdl_wrap.h>

//just for vscode indexer
#include <eigen3/Eigen/Dense>


namespace rct
{
class kchain : public kdl_wrap
{
public:
    kchain(std::string robot_description, std::string base_name, std::string ee_name, double grv);

    void get_status();

private:
    void compute_state_ee();
    void pos2ee();
    void vel2ee();
    void acc2ee();
    void acc2base();

    //aliases
    Eigen::Matrix<double, 3, 1> & x = status.frame.pos.segment<3>(0);
    Eigen::Matrix<double, 3, 1> & dx = status.frame.vel.segment<3>(0); 
    Eigen::Matrix<double, 3, 1> & ddx = status.frame.acc.segment<3>(0);

    Eigen::Matrix<double, 3, 1> & quat = status.frame.pos.segment<3>(2);
    Eigen::Matrix<double, 3, 1> & w = status.frame.vel.segment<3>(2); 
    Eigen::Matrix<double, 3, 1> & dw = status.frame.acc.segment<3>(2);

    Eigen::Matrix<double, 3, 1> & x_ee = status.frame_ee.pos.segment<3>(0);
    Eigen::Matrix<double, 3, 1> & dx_ee = status.frame_ee.vel.segment<3>(0); 
    Eigen::Matrix<double, 3, 1> & ddx_ee = status.frame_ee.acc.segment<3>(0);

    Eigen::Matrix<double, 3, 1> & quat_ee = status.frame_ee.pos.segment<3>(2);
    Eigen::Matrix<double, 3, 1> & w_ee = status.frame_ee.vel.segment<3>(2); 
    Eigen::Matrix<double, 3, 1> & dw_ee = status.frame_ee.acc.segment<3>(2);
    
    Eigen::Matrix<double, 3, 3> & R = status.R;
    Eigen::Matrix<double, 3, 3> & R_T = status.R.transpose();                

};

} // namespace rct