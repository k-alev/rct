#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <kchain.h>

namespace rct
{
class robot : public kchain
{
public:
    robot(std::string robot_description, std::string base_name, std::string ee_name, double grv);

    void read_commands();
    void send_commands();

    // template <class T>
    // void read_HI(T handle);

    // private:
};

} // namespace rct