#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

namespace rct
{

void KDL2EigenVec(const KDL::FrameVel &state_frame, Eigen::Matrix<double, 6, 1> &x, Eigen::Matrix<double, 6, 1> &dx, Eigen::Matrix<double, 3, 3> &R, Eigen::Quaterniond &Q);
void KDL2EigenVec(const KDL::Twist &twist, Eigen::Matrix<double, 6, 1> &vec);
void KDLRot2Mat(const KDL::Rotation &rot, Eigen::Matrix<double, 3, 3> &mat);
void unwrap_rotation(Eigen::Matrix<double, 6, 1> &x, Eigen::Matrix<double, 6, 1> &xold, std::vector<int> &counter);
void convert_jarray(const KDL::Chain &chain, const Eigen::MatrixXd &ddx, KDL::JntArray &var);
void convert_jarray(const KDL::Chain &chain, const KDL::JntArray &con, Eigen::MatrixXd &var);
void convert_wrench(const KDL::Wrench &con, Eigen::Matrix<double,6,1> &var);
void convert_wrench(const Eigen::Matrix<double,6,1> &con, KDL::Wrench &var);
Eigen::MatrixXd compute_JacInvPrd(const KDL::Chain &chain, const Eigen::MatrixXd&Jac, const Eigen::Matrix<double,6,1> &x);
Eigen::MatrixXd compute_JacInvPrd(const Eigen::MatrixXd &Jac, const Eigen::Matrix<double, 6, 1> &x);
Eigen::MatrixXd compute_JacInvPrd(const KDL::Chain &chain, const Eigen::MatrixXd &Jac, const Eigen::Matrix<double, 6, 1> &x, const double& lambda);
Eigen::Matrix<double, 3, 3> E(const Eigen::Quaterniond &q);
template <class T>
Eigen::Matrix<double, 3, 3> S(const T &w);

} // namespace rct