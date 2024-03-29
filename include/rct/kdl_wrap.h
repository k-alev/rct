#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_extras/chainJntToJacDotSolver.h>
#include <kdl_extras/chainfksolvervel_recursive.hpp>

#include <status.h>
#include <utils.h>



namespace rct
{
class kdl_wrap
{

public:
  kdl_wrap(std::string robot_description, std::string base_name, std::string ee_name, double grv = -9.81, bool ENABLE_LINKS = false);
  ~kdl_wrap();
  //Functions
  void get_inv_dynamics_cmd(const Eigen::MatrixXd &ddx, const std::string &opt, Eigen::MatrixXd &trq);
  void get_inv_dynamics_cmd(const Eigen::MatrixXd &ddx, const std::string &opt);
  void get_inv_dynamics_cmd(const Eigen::MatrixXd &ddx, Eigen::MatrixXd &trq);
  void get_inv_dynamics_cmd(const Eigen::MatrixXd &ddx);
  void get_inv_dynamics_cmd(const Eigen::MatrixXd &ddx, const std::string& opt, const double& lambda);
  void get_inv_dynamics_cmd(const Eigen::MatrixXd &ddx, const double& lambda);
  void get_joint_vel_cmd(const Eigen::MatrixXd &vel, const double &lambda);
  void get_joint_vel_cmd(const Eigen::MatrixXd &vel, const std::string& opt, const double& lambda);
  std::vector<Status> get_links_status();
  KDL::Chain get_kdl_chain();

protected:
  // Eigen declarations
  // Eigen::MatrixXd J;
  // Eigen::Matrix<double,6,1> dJdq;
  // Eigen::Matrix<double,3,3> R;
  // Eigen::MatrixXd q_conf;
  // Eigen::MatrixXd dq_conf;  
  // // Eigen::Matrix<double, 6, 1> jdqd;
  // // Eigen::Matrix<double, 6, 1> w_l_base;
  // // Eigen::MatrixXd jacInv;
  
  void set_state();
  void update();
  void update_start();
  Status status = Status();
  std::vector<Status> statuser;
  KDL::JntArray q;
  KDL::JntArray dq;
  KDL::Wrench ft_base;
  KDL::JntArray torque;
  KDL::FrameVel state_frame;
  std::vector<KDL::FrameVel> state_frames;


private:
  void init_solvers(std::string robot_description, std::string base_name, std::string ee_name);
  void compute_id_kdl();
  void compute_jac_kdl();
  void compute_jac();
  void compute_fk_kdl();
  void compute_fk();
  void compute_djac_kdl();
  void compute_djac();
  
  //KDL declarations
  KDL::ChainFkSolverVel_recursive_extras *fksolver;
  KDL::ChainIdSolver_RNE *idsolver;
  KDL::ChainJntToJacSolver *jacsolver;
  KDL::ChainJntToJacDotSolver *jacdotsolver;
  KDL::ChainJntToJacDotSolver *iksolver;
  // KDL::ChainDynParam *chDynParam;
  KDL::Chain chain;
  KDL::JntArrayVel q_dq_array;
  KDL::JntArray v;
  KDL::Jacobian jac;
  KDL::Twist jdot_qdot;
  KDL::Wrenches fext;
  // KDL::Wrench w_sensor_kdl;
  // KDL::Wrench w_base_kdl;

  //General declarations
  std::vector<int> cntr={0,0,0};
  std::string _robot_description; 
  std::string _base_name; 
  std::string _ee_name;
  double _g;
  Eigen::Matrix<double, 6, 1> xold;
  bool _ENABLE_LINKS;

};
} // namespace rct
