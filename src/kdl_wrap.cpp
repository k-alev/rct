
#include <kdl_wrap.h>

namespace rct
{
kdl_wrap::kdl_wrap(std::string robot_description, std::string base_name, std::string ee_name, double grv): 
_robot_description{robot_description}, _base_name{base_name}, _ee_name{ee_name}, _g{grv}
{
  init_solvers(_robot_description, _base_name, _ee_name);
  q.resize(chain.getNrOfJoints());
  dq.resize(chain.getNrOfJoints());
  q_dq_array.resize(chain.getNrOfJoints());

  jac.resize(chain.getNrOfJoints());
  J = jac.data;
  // jacInv.resize(chain.getNrOfJoints(),6);

  v.resize(chain.getNrOfJoints());
  torque.resize(chain.getNrOfJoints());
  fext.resize(chain.getNrOfSegments());

  q_conf.resize(chain.getNrOfJoints(),1);
  dq_conf.resize(chain.getNrOfJoints(),1);

}

kdl_wrap::~kdl_wrap()
{
	delete fksolver;
	delete idsolver;
	delete jacsolver;
	delete jacdotsolver;
	// delete chDynParam;
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
  J = jac.data;
}

void kdl_wrap::compute_djac_kdl()
{
  if(jacdotsolver->JntToJacDot(q_dq_array,jdot_qdot,-1) < 0)
	  throw std::runtime_error("KDL jacobian derivative solver failed.");
}

void kdl_wrap::compute_djac()
{
  compute_djac_kdl();
  KDL2EigenVec(jdot_qdot, dJdq);
}

void kdl_wrap::compute_fk_kdl()
{
  if (fksolver->JntToCart(q_dq_array, state_frame, -1) < 0)
    throw std::runtime_error("KDL forward kinematics solver failed.");
}

void kdl_wrap::compute_fk()
{
  convert_jarray(q, q_conf);
  convert_jarray(dq, dq_conf);
  compute_fk_kdl();
  KDL2EigenVec(state_frame, x, dx, R);
  unwrap_rotation(x, xold, cntr);
}

void kdl_wrap::update()
{
  update_state();
  compute_fk();
  compute_jac();
  compute_djac();
}

void kdl_wrap::update_state()
{
  std::cout << "Theoretically q,dq,f.. are updated" << std::endl;
  q_dq_array.q = q;
  q_dq_array.qdot = dq;
}

void kdl_wrap::update_start()
{
  update();
  // this->x = this->xold
  // and everything else that needs initialization at contoller.starting()
}

void kdl_wrap::get_command_id(const Eigen::MatrixXd &ddx, Eigen::MatrixXd trq)
{
  convert_jarray(ddx, v);
  compute_id_kdl();
  convert_jarray(torque, trq);
}

void kdl_wrap::convert_jarray(const Eigen::MatrixXd &con, KDL::JntArray &var)
{
  if (con.rows() != chain.getNrOfJoints() || con.cols() != 1)
    throw std::length_error("invalid matrix shape");
  var.data = con;
}

void kdl_wrap::convert_jarray(const KDL::JntArray &con, Eigen::MatrixXd &var)
{
  if (var.rows() != chain.getNrOfJoints() || var.cols() != 1)
    throw std::length_error("invalid matrix shape");
  var = con.data;
}

void kdl_wrap::KDL2EigenVec(const KDL::FrameVel &state_frame, Eigen::Matrix<double, 6, 1> &x, Eigen::Matrix<double, 6, 1> &dx, Eigen::Matrix<double, 3, 3> &R)
{
  double eta;
  for (unsigned int i = 0; i < 3; i++)
  {
    x(i) = state_frame.p.p[i];
    dx(i) = state_frame.p.v[i];
    dx(3 + i) = state_frame.M.w[i];
  }

  //state_frame.M.R.GetRPY(x(3), x(4), x(5));
  state_frame.M.R.GetQuaternion(x(3), x(4), x(5), eta);
  state_frame.M.R.GetQuaternion(state.pos(3), state.pos(4), state.pos(5), eta);
  // quat.x() = x(3);
  // quat.y() = x(4);
  // quat.z() = x(5);
  // quat.w() = eta;

  KDLRot2Mat(state_frame.M.R, R);
  return;
}

void kdl_wrap::KDL2EigenVec(const KDL::Twist &twist, Eigen::Matrix<double, 6, 1> &vec)
{
  vec(0) = twist.vel.x();
  vec(1) = twist.vel.y();
  vec(2) = twist.vel.z();
  vec(3) = twist.rot.x();
  vec(4) = twist.rot.y();
  vec(5) = twist.rot.z();

  return;
}

void kdl_wrap::unwrap_rotation(Eigen::Matrix<double, 6, 1> &x, Eigen::Matrix<double, 6, 1> &xold, std::vector<int> &counter)
{
  Eigen::Matrix<double, 6, 1> xt;
  for (unsigned int i = 0; i < 3; i++)
  {
    xt(i + 3) = x(i + 3) + counter[i] * 2.0 * 3.1415;
    if (xold(i + 3) - xt(i + 3) > 4)
      counter[i]++;
    else if (xold(i + 3) - xt(i + 3) < -4)
      counter[i]--;
    x(i + 3) = x(i + 3) + counter[i] * 2.0 * 3.1415;
    xold(i + 3) = x(i + 3);
  }
  return;
}

void kdl_wrap::KDLRot2Mat(const KDL::Rotation &rot, Eigen::Matrix<double, 3, 3> &mat)
{
  //should be implemented with eigen::map
  KDL::Vector vtmp = rot.UnitX();
  mat(0,0) = vtmp.x();
  mat(1,0) = vtmp.y();
  mat(2,0) = vtmp.z();
  vtmp = rot.UnitY();
  mat(0,1) = vtmp.x();
  mat(1,1) = vtmp.y();
  mat(2,1) = vtmp.z();
  vtmp = rot.UnitZ();
  mat(0,2) = vtmp.x();
  mat(1,2) = vtmp.y();
  mat(2,2) = vtmp.z();
}

void kdl_wrap::init_solvers(std::string robot_description, std::string base_name, std::string ee_name)
{
  KDL::Tree tree;

  if (!kdl_parser::treeFromString(robot_description, tree))
    throw std::runtime_error("Failed to construct KDL tree.");

  if (!tree.getChain(base_name, ee_name, chain)) //"m1n6s200_end_effector"
    throw std::runtime_error("Failed to get chain from KDL tree.");

  KDL::Vector g(0.0, 0.0, _g);

  if ((fksolver = new KDL::ChainFkSolverVel_recursive(chain)) == NULL)
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