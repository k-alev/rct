#include <iostream>
#include <kdl_wrap.h>
#include <kchain.h>
#include <string>
#include <fstream>
#include <streambuf>
#include <Eigen/Dense>



int main(int argc, char const *argv[])
{
    std::cout<<"Hello World!"<<std::endl;
    
    std::ifstream t("../myMico.xml");
    std::string str((std::istreambuf_iterator<char>(t)),
                 std::istreambuf_iterator<char>());
    
    
    std::string root = "root";
    std::string ee = "m1n6s200_end_effector";
    rct::kdl_wrap wrapper = rct::kdl_wrap(str, root, ee);
    wrapper.update();

    // test 1
    Eigen::Matrix<double, 6, 1> ddx_cmd;
    Eigen::Matrix<double, 6, 1> trq;
    wrapper.get_command_id(ddx_cmd, trq);
    // robot.set_command(trq)

    rct::kchain myChain = rct::kchain(str, root, ee, 90);
    
    return 0;
}