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
    void get_state_ee();
    void get_state();
    void get_rotation();

// private:

};

} // namespace rct