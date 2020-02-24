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
    void read_sensors(const T& xhandle)
    {
        read_from_robot(xhandle);
        set_state();
    };

    template <class T>
    void send_commands(const T& xhandle)
    {
        write_to_robot(xhandle);
        //other necessary stuff
    };

    template <class T>
    void write_to_robot(const T& handle);
 
    template <class T>
    void read_from_robot(const T& handle);

};

} // namespace rct