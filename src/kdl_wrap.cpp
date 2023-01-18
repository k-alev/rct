
#include <kdl_wrap.h>

namespace rct
{
kdl_wrap::kdl_wrap(std::string robot_description, std::string base_name, std::string ee_name, double grv, bool ENABLE_LINKS) 
: _robot_description{robot_description}, _base_name{base_name}, _ee_name{ee_name}, _g{grv}, _ENABLE_LINKS{ENABLE_LINKS}
{
  init_solvers(_robot_description, _base_name, _ee_name);
  q.resize(chain.getNrOfJoints());
  dq.resize(chain.getNrOfJoints());
  q_dq_array.resize(chain.getNrOfJoints());

  jac.resize(chain.getNrOfJoints());
  status.J = jac.data;
  // jacInv.resize(chain.getNrOfJoints(),6);

  v.resize(chain.getNrOfJoints());
  torque.resize(chain.getNrOfJoints());
  fext.resize(chain.getNrOfSegments());

  status.q_conf.resize(chain.getNrOfJoints(), 1);
  status.dq_conf.resize(chain.getNrOfJoints(), 1);

  state_frames.resize(chain.getNrOfSegments());
  statuser.resize(chain.getNrOfSegments());
}

kdl_wrap::~kdl_wrap()
{
  delete fksolver;
  delete idsolver;
  delete jacsolver;
  delete jacdotsolver;
  // delete chDynParam;
}

std::vector<Status> kdl_wrap::get_links_status()
{
  if (!_ENABLE_LINKS)
    throw std::runtime_error("Full chain solver is not enabled. ENABLE_LINKS param should be set to TRUE.");
  return statuser;
}

KDL::Chain kdl_wrap::get_kdl_chain()
{
  return chain;
}

void kdl_wrap::compute_id_kdl()
{
  if (idsolver->CartToJnt(q, dq, v, fext, torque) < 0)
    throw std::runtime_error("KDL inverse dynamics solver failed.");
}

void kdl_wrap::compute_jac_kdl()
{
  if (jacsolver->JntToJac(q, jac, -1) < 0)
    throw std::runtime_error("KDL jacobian solver failed.");
}

void kdl_wrap::compute_jac()
{
  compute_jac_kdl();
  status.J = jac.data;
}

void kdl_wrap::compute_djac_kdl()
{
  if (jacdotsolver->JntToJacDot(q_dq_array, jdot_qdot, -1) < 0)
    throw std::runtime_error("KDL jacobian derivative solver failed.");
}

void kdl_wrap::compute_djac()
{
  compute_djac_kdl();
  KDL2EigenVec(jdot_qdot, status.dJdq);
}

void kdl_wrap::compute_fk_kdl()
{
  if(!_ENABLE_LINKS)
  {
    if (fksolver->JntToCart(q_dq_array, state_frame, -1) < 0)
      throw std::runtime_error("KDL forward kinematics solver failed.");
  }
  else
  {
    if (fksolver->JntToCart(q_dq_array, state_frames, -1) < 0)
      throw std::runtime_error("KDL forward kinematics solver failed.");
    state_frame = state_frames.back();
  }
}

void kdl_wrap::compute_fk()
{
  // convert_jarray(chain, q, status.q_conf);
  // convert_jarray(chain, dq, status.dq_conf);
  compute_fk_kdl();
  KDL2EigenVec(state_frame, status.frame.pos, status.frame.vel, status.R, status.quat);
  if(_ENABLE_LINKS)
  {
    for(int i =0; i<statuser.size(); i++)
          KDL2EigenVec(state_frames[i], statuser[i].frame.pos, statuser[i].frame.vel, statuser[i].R, statuser[i].quat);
  }
  // unwrap_rotation(status.frame.pos, xold, cntr);
}

void kdl_wrap::update()
{
  // set_state(); //moved to robot::read()
  compute_fk();
  compute_jac();
  compute_djac();
}

void kdl_wrap::set_state()
{
  //is responsible to assign measurements to the corresponding internal variables
  //should run in robot::read_sensor_handles()
  // std::cout << "Theoretically q,dq,f.. are updated" << std::endl;
  convert_jarray(chain, q, status.q_conf);
  convert_jarray(chain, dq, status.dq_conf);
  q_dq_array.q = q;
  q_dq_array.qdot = dq;
  convert_wrench(ft_base, status.frame.wrench);
}

void kdl_wrap::update_start()
{
  update();
  this->status.frame.pos = this->xold;
  // and everything else that needs initialization at contoller.starting()
}

void kdl_wrap::get_inv_dynamics_cmd(const Eigen::MatrixXd &ddx, Eigen::MatrixXd &trq)
{
  std::string opt = "cartesian";
  get_inv_dynamics_cmd(ddx, opt, trq);
}

void kdl_wrap::get_inv_dynamics_cmd(const Eigen::MatrixXd &ddx, const std::string &opt, Eigen::MatrixXd &trq)
{
  if ((trq.rows() != chain.getNrOfJoints() || trq.cols() != 1))
    throw std::length_error("invalid matrix shape");
  // Eigen::MatrixXd v_conf = compute_JacInvPrd(chain, status.J, ddx - status.dJdq);
  // convert_jarray(chain, ddx, v);
  // compute_id_kdl();
  get_inv_dynamics_cmd(ddx, opt);
  convert_jarray(chain, torque, trq);
  //should implement pinv solver too (perhaps overload with pseudo-inverse as arg)
}

void kdl_wrap::get_inv_dynamics_cmd(const Eigen::MatrixXd &ddx)
{
  std::string opt = "cartesian";
  get_inv_dynamics_cmd(ddx, opt);
}

void kdl_wrap::get_inv_dynamics_cmd(const Eigen::MatrixXd &ddx, const double &lambda)
{
  std::string opt = "cartesian";
  get_inv_dynamics_cmd(ddx, opt, lambda);
}

void kdl_wrap::get_inv_dynamics_cmd(const Eigen::MatrixXd &ddx, const std::string &opt)
{
  double lambda = 0.0;
  get_inv_dynamics_cmd(ddx, opt, lambda);
}

void kdl_wrap::get_inv_dynamics_cmd(const Eigen::MatrixXd &ddx, const std::string &opt, const double &lambda)
{
  // TODO: Add a more robust implementation for opt checking
  Eigen::MatrixXd v_conf;
  if (opt == "cartesian")
  {
    if ((ddx.rows() != 6 || ddx.cols() != 1))
      throw std::length_error("invalid matrix shape");
    v_conf = compute_JacInvPrd(chain, status.J, ddx - status.dJdq, lambda);
    // v_conf = (status.J.transpose()*status.J).inverse()*status.J.transpose()*(ddx - status.dJdq);
    // v_conf = status.J.transpose()*(status.J*status.J.transpose()).inverse()*(ddx - status.dJdq);
  }
  else
  {
    if ((ddx.rows() != chain.getNrOfJoints() || ddx.cols() != 1))
      throw std::length_error("invalid matrix shape");
    v_conf = ddx;
  }
  convert_jarray(chain, v_conf, v);
  compute_id_kdl();
}

void kdl_wrap::get_joint_vel_cmd(const Eigen::MatrixXd &vel, const double &lambda)
{
  // TODO: Similar implementation with get_inv_dynamics_cmd
  std::string opt = "cartesian";
  get_joint_vel_cmd(vel, opt, lambda);
}

void kdl_wrap::get_joint_vel_cmd(const Eigen::MatrixXd &vel, const std::string &opt, const double &lambda)
{
  // TODO: Similar implementation with get_inv_dynamics_cmd
  // Eigen::MatrixXd qdot = status.J.inverse() * vel;
  if (opt == "cartesian")
  {
    if ((vel.rows() != 6 || vel.cols() != 1))
      throw std::length_error("invalid matrix shape");
    Eigen::MatrixXd qdot = compute_JacInvPrd(chain, status.J, vel, lambda);
    convert_jarray(chain, qdot, torque);
  }
  else
  {
    if ((vel.rows() != chain.getNrOfJoints() || vel.cols() != 1))
      throw std::length_error("invalid matrix shape");
    convert_jarray(chain, vel, torque);
  }
}

void kdl_wrap::init_solvers(std::string robot_description, std::string base_name, std::string ee_name)
{
  KDL::Tree tree;

  if (!kdl_parser::treeFromString(robot_description, tree))
    throw std::runtime_error("Failed to construct KDL tree.");

  if (!tree.getChain(base_name, ee_name, chain)) //"m1n6s200_end_effector"
    throw std::runtime_error("Failed to get chain from KDL tree.");

  KDL::Vector g(0.0, 0.0, _g);

  if ((fksolver = new KDL::ChainFkSolverVel_recursive_extras(chain)) == NULL)
    throw std::runtime_error("Failed to create ChainFkSolverVel_recursive.");

  if ((jacsolver = new KDL::ChainJntToJacSolver(chain)) == NULL)
    throw std::runtime_error("Failed to create ChainJntToJacSolver.");

  if ((idsolver = new KDL::ChainIdSolver_RNE(chain, g)) == NULL)
    throw std::runtime_error("Failed to create ChainIDSolver_RNE.");

  if ((jacdotsolver = new KDL::ChainJntToJacDotSolver(chain)) == NULL)
    throw std::runtime_error("Failed to create ChainJntToJacDotSolver.");

  // if ((chDynParam = new KDL::ChainDynParam(chain, g)) == NULL)
  // {
  //   std::cout << "Failed to create Chain Dyn. Params." << std::endl;
  //   throw;
  // }

}

} // namespace rct