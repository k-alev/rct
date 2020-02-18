#include <kchain.h>

namespace rct
{
kchain::kchain(std::string robot_description, std::string base_name, std::string ee_name, double grv) 
: kdl_wrap{robot_description, base_name, ee_name, grv}
{
    std::cout << "Chain Constructed!!!" << std::endl;
}

void kchain::get_status()
{
    compute_state_ee();
}

void kchain::compute_state_ee()
{
    // Rt = R.transpose()
    pos2ee();
    vel2ee();
    acc2ee();
}

void kchain::pos2ee()
{
    x_ee = R_T*x;
    // // quaternions are missing
}

void kchain::vel2ee()
{   
    dx_ee = R_T*(dx - w.cross(x));
    w_ee = R_T*w;
}

void kchain::acc2ee()
{
    ddx_ee = R_T*(ddx -2*w.cross(R*dx_ee) - dw.cross(x) -  w.cross(w.cross(x)));
    dw_ee = R_T*(dw - w.cross(w));
}

void kchain::acc2base()
{
    ddx = R*ddx_ee -2*w.cross(R*dx) - dw.cross(x) -  w.cross(w.cross(x));
    dw<3>(2) = R*dw_ee + w.cross(w);
}


} // namespace rct