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
  ForceTorqueSensorHandle(const std::string &name,
                          const std::string &frame_id,
                          const double *force,
                          const double *torque)
      : name_(name),
        frame_id_(frame_id),
        force_(force),
        torque_(torque)
  {
  }

  // keep non-const version for binary compatibility
  ForceTorqueSensorHandle(const std::string &name,
                          const std::string &frame_id,
                          double *force,
                          double *torque)
      : name_(name),
        frame_id_(frame_id),
        force_(force),
        torque_(torque)
  {
  }

  std::string getName() const { return name_; }
  std::string getFrameId() const { return frame_id_; }
  const double *getForce() const { return force_; }
  const double *getTorque() const { return torque_; }

private:
  std::string name_;
  std::string frame_id_;
  const double *force_;
  const double *torque_;
};

template <class T>
void rct::robot::read_from_robot(const T &handle)
{
  std::cout << "reading from robot unspecialized" << std::endl;
  // q(0) = (3.1415/180)*274.91;
  // q(1) = (3.1415/180)*174.98;
  // q(2) = (3.1415/180)*78.99;
  // q(3) = (3.1415/180)*243.17;
  // q(4) = (3.1415/180)*83.78;
  // q(5) = (3.1415/180)*75.28;
}

namespace rct
{
template <>
void robot::read_from_robot<ForceTorqueSensorHandle>(const ForceTorqueSensorHandle &handle)
{
  const double *tmp_frc, *tmp_trq;

  tmp_frc = handle.getForce();
  tmp_trq = handle.getTorque();

  for (unsigned int i = 0; i < 3; i++)
  {
    ft.force(i) = tmp_frc[i];
    ft.torque(i) = tmp_trq[i];
  }

  std::cout << "Reading HI " << this->ft.force(2) << std::endl;
}
} // namespace rct

template <class T>
void rct::robot::write_to_robot(T &handle)
{
  std::cout << "writing to robot " << std::endl;
}

namespace rct
{
template <>
void robot::write_to_robot<int>(int &handle)
{
  std::cout << "writing to specialized robot " << handle << std::endl;
}
} // namespace rct



int main(int argc, char const *argv[])
{
  std::cout << "Hello World!" << std::endl;

  std::ifstream t("../myMico.xml");
  std::string str((std::istreambuf_iterator<char>(t)),
                  std::istreambuf_iterator<char>());

  std::string root = "root";
  std::string ee = "m1n6s200_end_effector";
  rct::kdl_wrap wrapper = rct::kdl_wrap(str, root, ee);
  // wrapper.update();

  // test 1
  // Eigen::Matrix<double, 6, 1> ddx_cmd;
  // Eigen::Matrix<double, 6, 1> trq;
  // wrapper.get_inv_dynamics_cmd(ddx_cmd, trq);

  // rct::kchain myChain = rct::kchain(str, root, ee, -10);
  // // myChain.update();
  // rct::Status junkStatus;
  // junkStatus = myChain.get_status("ee");

  std::cout << "initializing robot..." << std::endl;
  rct::robot myRobot = rct::robot(str, root, ee, -10);

  // myRobot.read_from_robot<std::string>("like now");
  myRobot.read_sensor_handles<std::string>("read_sensors");
  rct::Status junkStatus;
  junkStatus = myRobot.get_status("ee");
  std::cout << "Pos: " <<junkStatus.frame.pos(0)<<", "<<junkStatus.frame.pos(1)<<", "<<junkStatus.frame.pos(2)<< std::endl;
  std::cout << "Or: " <<junkStatus.frame.pos(3)<<", "<<junkStatus.frame.pos(4)<<", "<<junkStatus.frame.pos(5)<< std::endl;
  std::cout << "PosEE: " <<junkStatus.frame_ee.pos(0)<<", "<<junkStatus.frame_ee.pos(1)<<", "<<junkStatus.frame_ee.pos(2)<< std::endl;
  std::cout << "OrEE: " <<junkStatus.frame_ee.pos(3)<<", "<<junkStatus.frame_ee.pos(4)<<", "<<junkStatus.frame_ee.pos(5)<< std::endl;
  std::cout << "eta: " <<junkStatus.quat.w()<<std::endl;

  // const double *test1, *test2;
  // test1 = (const double *)malloc(3);
  // test2 = (const double *)malloc(3);
  // ForceTorqueSensorHandle ft_handle = ForceTorqueSensorHandle("test", "test2", test1, test2);
  // myRobot.read_sensors<ForceTorqueSensorHandle>(ft_handle);



  // #####################CONTROL FLOW PSEUDO CODE############################## //

  //control flow 1
  // myRobot.read_sensors();
  // cur_status = myRobot.get_status("ee"); // with options, phaps update status
  // cmd_acc_ee = controller.control(cur_status); //command is accelerations perhaps in ee_frame
  // cmd_acc = myRobot.acc2base(cmd_acc_ee); //needed only if cmd acc is in ee_frame
  // myRobot.get_inv_dynamics_cmd(cmd_acc, torque);
  // myRobot.send_commands(torque); // probably arg is not needed

  //control flow 2
  // myRobot.read_sensors();
  // myRobot.get_status(cur_status); // with options, phaps update status
  // controller.control(cur_status, time, command); //command is velocities perhaps in ee_frame (should we leave the controller deal with integration/ differentiation??)
  // myRobot.vel2base(command, vel_base) //needed only if cmd vel is in ee_frame
  // myRobot.get_admittance_cmd(vel_base, cmd_dq); //should change name to get_jvel_cmd
  // myRobot.send_commands(cmd_dq);

  //control flow 3
  // myRobot.read_sensors();
  // myRobot.get_status(cur_status); // with options, phaps update status
  // controller.control(cur_status, command); //command is position perhaps in ee_frame
  // myRobot.pos2base(command, pos_base) //needed only if cmd pos is in ee_frame
  // myRobot.get_IK_cmd(pos_base, cmd_dq);
  // myRobot.send_commands(cmd_dq);

  return 0;
}