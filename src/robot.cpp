#include <robot.h>

namespace rct
{
robot::robot(std::string robot_description, std::string base_name, std::string ee_name, double grv)
    : kchain{robot_description, base_name, ee_name, grv}
{
    std::cout << "Robot Constructed!!!" << std::endl;
}

void robot::read_commands()
{
    // read_custom_robot();
    set_state();
}

// template <class T>
// void robot::read_HI(T handle)
// {
//     for (unsigned int i = 0; i < handle.size(); i++)
//     {
//         q(i) = handle[i].getPosition();
//         dq(i) = handle[i].getVelocity();
//         // jnt_trq(i) = handle[i].getEffort();
//     }
// }

// template <class T>
// void robot::read_HI(T handle)
// {
//     // ft.force.data = handle.getForce();
// 	// ft.torque.data = handle.getTorque();
//     std::cout<<"Reading HI"<<std::endl;
// }

} // namespace rct