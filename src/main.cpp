#include <iostream>
#include <kdl_wrap.h>
#include <kchain.h>
#include <robot.h>
#include <string>
#include <fstream>
#include <streambuf>
#include <Eigen/Dense>


class ForceTorqueSensorHandle
{
public:
  ForceTorqueSensorHandle() : name_(""), frame_id_(""), force_(0), torque_(0) {}

  /**
   * \param name The name of the sensor
   * \param frame_id The reference frame to which this sensor is associated
   * \param force A pointer to the storage of the force value: a triplet (x,y,z)
   * \param torque A pointer to the storage of the torque value: a triplet (x,y,z)
   *
   */
  ForceTorqueSensorHandle(const std::string& name,
                          const std::string& frame_id,
                          const double* force,
                          const double* torque)
    : name_(name),
      frame_id_(frame_id),
      force_(force),
      torque_(torque)
  {}

  // keep non-const version for binary compatibility
  ForceTorqueSensorHandle(const std::string& name,
                          const std::string& frame_id,
                          double* force,
                          double* torque)
    : name_(name),
      frame_id_(frame_id),
      force_(force),
      torque_(torque)
  {}

  std::string getName()     const {return name_;}
  std::string getFrameId()  const {return frame_id_;}
  const double* getForce()  const {return force_;}
  const double* getTorque() const {return torque_;}

private:
  std::string name_;
  std::string frame_id_;
  const double* force_;
  const double* torque_;
};

int main(int argc, char const *argv[])
{
    std::cout<<"Hello World!"<<std::endl;
    
    std::ifstream t("../myMico.xml");
    std::string str((std::istreambuf_iterator<char>(t)),
                 std::istreambuf_iterator<char>());
    
    
    std::string root = "root";
    std::string ee = "m1n6s200_end_effector";
    rct::kdl_wrap wrapper = rct::kdl_wrap(str, root, ee);
    // wrapper.update();

    // test 1
    Eigen::Matrix<double, 6, 1> ddx_cmd;
    Eigen::Matrix<double, 6, 1> trq;
    wrapper.get_inv_dynamics_cmd(ddx_cmd, trq);
    // robot.set_command(trq)

    rct::kchain myChain = rct::kchain(str, root, ee, -10);
    // myChain.update();
    myChain.get_status();


    std::cout<<"initializing robot..."<<std::endl;
    rct::robot myRobot = rct::robot(str, root, ee, -10);
    myRobot.get_status();
    // ForceTorqueSensorHandle ft_handle = ForceTorqueSensorHandle();
    // myRobot.read_HI<ForceTorqueSensorHandle>(ft_handle);
    
    return 0;
}