#include <kchain.h>

namespace rct
{
kchain::kchain(std::string robot_description, std::string base_name, std::string ee_name, double grv)
    : kdl_wrap{robot_description, base_name, ee_name, grv}
{
    std::cout << "Chain Constructed!!!" << std::endl;
}

Status kchain::get_status(std::string options)
{
    update();
    R_T = status.R.transpose();
    if (1)
        compute_state_ee(); // optional, if not it just returns the copied kdl status
    return status;
}

Status kchain::get_status_start(std::string options)
{
    update_start();
    R_T = status.R.transpose();
    if (1)
        compute_state_ee(); // optional, if not it just returns the copied kdl status
    return status;
}

void kchain::compute_state_ee()
{
    pos2ee();
    vel2ee();
    wrench2ee();
    // acc2ee();
}

void kchain::wrench2ee()
{
    frc_ee = R_T * frc;
    trq_ee = R_T * trq;
}

void kchain::wrench2base()
{
    frc = R * frc_ee;
    trq = R * trq_ee;
}

void kchain::pos2ee()
{
    x_ee = R_T * x;
    quat_ee(0) = 0;
    quat_ee(1) = 0;
    quat_ee(2) = 0;
}

void kchain::vel2ee()
{
    dx_ee = R_T * (dx - w.cross(x));
    w_ee = R_T * w;
}

// void kchain::vel2base()
// {
//     dx = R * dx_ee + w.cross(x);
//     w = R * w_ee;
// }

void kchain::acc2ee()
{
    // TODO: Implementation does not make sense. See also acc2base
    ddx_ee = R_T * (ddx - 2 * w.cross(R * dx_ee) - dw.cross(x) - w.cross(w.cross(x)));
    dw_ee = R_T * (dw - w.cross(w));
}

// void kchain::acc2base()
// {
//     ddx = R * ddx_ee + 2 * w.cross(R * dx_ee) + dw.cross(x) + w.cross(w.cross(x));
//     dw = R * dw_ee + w.cross(w);
// }

Eigen::VectorXd kchain::acc2base(const Eigen::VectorXd &ee_acc)
{
    if (ee_acc.rows() != 6)
        throw std::length_error("invalid matrix shape");
    Eigen::VectorXd _ddx_ee = ee_acc.segment<3>(0);
    Eigen::VectorXd _dw_ee = ee_acc.segment<3>(3);
    Eigen::Matrix<double, 6, 1> base_acc;
    // base_acc.segment<3>(0) = R * _ddx_ee + 2 * w.cross(R * dx_ee) + dw.cross(x) + w.cross(w.cross(x)); //dw should be computed first
    base_acc.segment<3>(3) = R * _dw_ee + w.cross(w);
    base_acc.segment<3>(0) = R * _ddx_ee + 2 * w.cross(R * dx_ee) + base_acc.segment<3>(3).cross(x) + w.cross(w.cross(x));

    return base_acc;
}

Eigen::VectorXd kchain::vel2base(const Eigen::VectorXd &ee_vel)
{
    if (ee_vel.rows() != 6)
        throw std::length_error("invalid matrix shape");
    Eigen::VectorXd _dx_ee = ee_vel.segment<3>(0);
    Eigen::VectorXd _w_ee = ee_vel.segment<3>(3);
    Eigen::Matrix<double, 6, 1> base_vel;
    base_vel.segment<3>(0) = R * _dx_ee + w.cross(x);
    base_vel.segment<3>(3) = R * _w_ee;

    return base_vel;
}

} // namespace rct