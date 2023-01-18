#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <kchain.h>

namespace rct
{
class robot : public kchain
{
public:
    robot(std::string robot_description, std::string base_name, std::string ee_name, double grv, bool ENABLE_LINKS = false)
        : kchain{robot_description, base_name, ee_name, grv, ENABLE_LINKS}
    {
        std::cout << "Robot Constructed!!!" << std::endl;
    };

    // template <class T>
    // void read_sensors(const T &xhandle)
    // {
    //     read_from_robot(xhandle);
    //     set_state();
    // };

    void read_sensor_handles()
    {
        set_state();
    }

    template <class T, class... Types>
    void read_sensor_handles(T T1, Types... T2)
    {
        read_from_robot<T>(T1);
        read_sensor_handles(T2...);
    }

    template <class T>
    void send_commands(T &xhandle)
    {
        write_to_robot<T>(xhandle);
        //other necessary stuff
    };

    template <class T>
    void write_to_robot(T &handle);

    template <class T>
    void read_from_robot(const T &handle);

};

} // namespace rct
