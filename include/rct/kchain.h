#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <kdl_wrap.h>

namespace rct
{
class kchain : public kdl_wrap
{
public:
    kchain(std::string robot_description, std::string base_name, std::string ee_name, double grv);

    Status get_status(std::string options);
    Status get_status_start(std::string options);
    void acc2ee();
    Eigen::VectorXd acc2base(const Eigen::VectorXd &ee_acc);
    Eigen::VectorXd vel2base(const Eigen::VectorXd &ee_vel);
    void pos2base();

private:
    void compute_state_ee();
    void pos2ee();
    void vel2ee();
    void wrench2ee();
    void wrench2base();
    //perhaps unnecessary
    void acc2base();
    void vel2base();

    //aliases
    Eigen::Ref<Eigen::Matrix<double, 3, 1>> x = status.frame.pos.segment<3>(0);
    Eigen::Ref<Eigen::Matrix<double, 3, 1>> dx = status.frame.vel.segment<3>(0);
    Eigen::Ref<Eigen::Matrix<double, 3, 1>> ddx = status.frame.acc.segment<3>(0);
    Eigen::Ref<Eigen::Matrix<double, 3, 1>> frc = status.frame.wrench.segment<3>(0);

    Eigen::Ref<Eigen::Matrix<double, 3, 1>> quat = status.frame.pos.segment<3>(3);
    Eigen::Ref<Eigen::Matrix<double, 3, 1>> w = status.frame.vel.segment<3>(3);
    Eigen::Ref<Eigen::Matrix<double, 3, 1>> dw = status.frame.acc.segment<3>(3);
    Eigen::Ref<Eigen::Matrix<double, 3, 1>> trq = status.frame.wrench.segment<3>(3);

    Eigen::Ref<Eigen::Matrix<double, 3, 1>> x_ee = status.frame_ee.pos.segment<3>(0);
    Eigen::Ref<Eigen::Matrix<double, 3, 1>> dx_ee = status.frame_ee.vel.segment<3>(0);
    Eigen::Ref<Eigen::Matrix<double, 3, 1>> ddx_ee = status.frame_ee.acc.segment<3>(0);
    Eigen::Ref<Eigen::Matrix<double, 3, 1>> frc_ee = status.frame.wrench.segment<3>(0);

    Eigen::Ref<Eigen::Matrix<double, 3, 1>> quat_ee = status.frame_ee.pos.segment<3>(3);
    Eigen::Ref<Eigen::Matrix<double, 3, 1>> w_ee = status.frame_ee.vel.segment<3>(3);
    Eigen::Ref<Eigen::Matrix<double, 3, 1>> dw_ee = status.frame_ee.acc.segment<3>(3);
    Eigen::Ref<Eigen::Matrix<double, 3, 1>> trq_ee = status.frame.wrench.segment<3>(3);

    Eigen::Ref<Eigen::Matrix<double, 3, 3>> R = status.R;
    Eigen::Matrix<double, 3, 3> R_T;
};

} // namespace rct