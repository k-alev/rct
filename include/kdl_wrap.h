#pragma once

#include <iostream>
#include <Eigen/Dense>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <chainJntToJacDotSolver.h>

#include <status.h>

namespace rct
{
class kdl_wrap
{

public:
  kdl_wrap(std::string robot_description, std::string base_name, std::string ee_name, double grv = -9.81);
  ~kdl_wrap();
  //Functions
  void update();
  void update_start();
  void get_inv_dynamics_cmd(const Eigen::MatrixXd &ddx, Eigen::MatrixXd trq);
  void get_admittance_cmd(const Eigen::MatrixXd &vel, Eigen::MatrixXd qdot);

protected:
  // Eigen declarations
  // Eigen::MatrixXd J;
  // Eigen::Matrix<double,6,1> dJdq;
  // Eigen::Matrix<double,3,3> R;
  // Eigen::MatrixXd q_conf;
  // Eigen::MatrixXd dq_conf;  
  // // Eigen::Quaternion quat = Eigen::Quaternion(0,0,0,1);
  // // Eigen::Matrix<double, 6, 1> jdqd;
  // // Eigen::Matrix<double, 6, 1> w_l_base;
  // // Eigen::MatrixXd jacInv;

  // // Other
  Status status = Status();
  // State state = State();
  // State state_ee = State();

private:
  void init_solvers(std::string robot_description, std::string base_name, std::string ee_name);
  void compute_id_kdl();
  void compute_jac_kdl();
  void compute_jac();
  void compute_fk_kdl();
  void compute_fk();
  void compute_djac_kdl();
  void compute_djac();
  
  void update_state();
  //auta edw mporoun na ginoun utils h allh class...den exoun sxesh me auth tn class
  void KDL2EigenVec(const KDL::FrameVel &state_frame, Eigen::Matrix<double, 6, 1> &x, Eigen::Matrix<double, 6, 1> &dx, Eigen::Matrix<double, 3, 3> &R);
  void KDL2EigenVec(const KDL::Twist &twist, Eigen::Matrix<double, 6, 1> &vec);
  void KDLRot2Mat(const KDL::Rotation &rot, Eigen::Matrix<double, 3, 3> &mat);
  void unwrap_rotation(Eigen::Matrix<double, 6, 1> &x, Eigen::Matrix<double, 6, 1> &xold, std::vector<int> &counter);
  void convert_jarray(const Eigen::MatrixXd &ddx, KDL::JntArray &var);
  void convert_jarray(const KDL::JntArray &con, Eigen::MatrixXd &var);
  
  //KDL declarations
  KDL::ChainFkSolverVel_recursive *fksolver;
  KDL::ChainIdSolver_RNE *idsolver;
  KDL::ChainJntToJacSolver *jacsolver;
  KDL::ChainJntToJacDotSolver *jacdotsolver;
  // KDL::ChainDynParam *chDynParam;
  KDL::Chain chain;
  KDL::JntArray q;
  KDL::JntArray dq;
  KDL::JntArrayVel q_dq_array;
  KDL::JntArray v;
  KDL::JntArray torque;
  KDL::Jacobian jac;
  KDL::Twist jdot_qdot;
  KDL::FrameVel state_frame;
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

};
} // namespace rct
