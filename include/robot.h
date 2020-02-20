#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <kchain.h>

namespace rct
{
class robot : public kchain
{
public:
    robot(std::string robot_description, std::string base_name, std::string ee_name, double grv)
        : kchain{robot_description, base_name, ee_name, grv}
    {
        std::cout << "Robot Constructed!!!" << std::endl;
    };

    template <class T>
    void read_sensors(T xhandle)
    {
        read_from_robot(xhandle);
        set_state();
    };

    template <class T>
    void send_commands(T xhandle)
    {
        write_to_robot(xhandle);
        //other necessary stuff
    };

    template <class T>
    void write_to_robot(T handle);

    //perhaps needs to be implemented with variadic templates to take multiple handles as input 
    template <class T>
    void read_from_robot(T handle);

};

} // namespace rct