#include <kchain.h>

namespace rct
{
kchain::kchain(std::string robot_description, std::string base_name, std::string ee_name, double grv) 
: kdl_wrap{robot_description, base_name, ee_name, grv}
{
    std::cout << "Constructed!!!" << std::endl;
}

} // namespace rct