#include <kchain.h>

namespace rct
{
kchain::kchain(std::string robot_description, std::string base_name, std::string ee_name, double grv) 
: kdl_wrap{robot_description, base_name, ee_name, grv}
{
    std::cout << "Chain Constructed!!!" << std::endl;
}

void kchain::get_state_ee()
{
    // Rt = R.transpose()
    pos2ee();
    vel2ee();
    acc2ee();
}

void kchain::pos2ee()
{
    state_ee.pos.segment<3>(0) = R.transpose()*state.pos.segment<3>(0);
    // quaternions are missing
}

void kchain::vel2ee()
{   
    state_ee.vel.segment<3>(0) = R.transpose()*(state.vel.segment<3>(0) - state.vel.segment<3>(2).cross(state.pos.segment<3>(0)));
    state_ee.vel.segment<3>(2) = R.transpose()*state.vel.segment<3>(2);
}

void kchain::acc2ee()
{
    state_ee.acc.segment<3>(0) = R.transpose()*(state.acc.segment<3>(0) -2*state.vel.segment<3>(2).cross(R*state_ee.vel.segment<3>(0)) 
     - state.acc.segment<3>(2).cross(state.pos.segment<3>(2)) -  state.vel.segment<3>(2).cross(state.vel.segment<3>(2).cross(state.pos.segment<3>(0))));
    state_ee.acc.segment<3>(2) = R.transpose()*(state.acc.segment<3>(2) - state.vel.segment<3>(2).cross(state.vel.segment<3>(2)));
}

void kchain::acc2base()
{
    state.acc.segment<3>(0) = R*state_ee.acc.segment<3>(0) -2*state.vel.segment<3>(2).cross(R*state_ee.vel.segment<3>(0)) 
     - state.acc.segment<3>(2).cross(state.pos.segment<3>(2)) -  state.vel.segment<3>(2).cross(state.vel.segment<3>(2).cross(state.pos.segment<3>(0)));

    state.acc.segment<3>(2) = R*state_ee.acc.segment<3>(2) + state.vel.segment<3>(2).cross(state.vel.segment<3>(2));
}


} // namespace rct