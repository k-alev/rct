#include <utils.h>
#include <iostream>

namespace rct
{
void convert_jarray(const KDL::Chain &chain, const Eigen::MatrixXd &con, KDL::JntArray &var)
{
    if (con.rows() != chain.getNrOfJoints() || con.cols() != 1)
        throw std::length_error("invalid matrix shape");
    var.data = con;
}

void convert_jarray(const KDL::Chain &chain, const KDL::JntArray &con, Eigen::MatrixXd &var)
{
    if (var.rows() != chain.getNrOfJoints() || var.cols() != 1)
        throw std::length_error("invalid matrix shape");
    var = con.data;
}

void convert_wrench(const KDL::Wrench &con, Eigen::Matrix<double, 6, 1> &var)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        var(i) = con.force(i);
        var(i + 3) = con.torque(i);
    }
}

void convert_wrench(const Eigen::Matrix<double, 6, 1> &con, KDL::Wrench &var)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        var.force(i) = con(i);
        var.torque(i) = con(i + 3);
    }
}

void KDL2EigenVec(const KDL::FrameVel &state_frame, Eigen::Matrix<double, 6, 1> &x, Eigen::Matrix<double, 6, 1> &dx, Eigen::Matrix<double, 3, 3> &R, Eigen::Quaterniond &Q)
{
    // double eta;
    for (unsigned int i = 0; i < 3; i++)
    {
        x(i) = state_frame.p.p[i];
        dx(i) = state_frame.p.v[i];
        dx(3 + i) = state_frame.M.w[i];
    }

    // state_frame.M.R.GetRPY(x(3), x(4), x(5));
    state_frame.M.R.GetQuaternion(x(3), x(4), x(5), Q.w());
    Q.x() = x(3);
    Q.y() = x(4);
    Q.z() = x(5);

    KDLRot2Mat(state_frame.M.R, R);
    return;
}

void KDL2EigenVec(const KDL::Twist &twist, Eigen::Matrix<double, 6, 1> &vec)
{
    vec(0) = twist.vel.x();
    vec(1) = twist.vel.y();
    vec(2) = twist.vel.z();
    vec(3) = twist.rot.x();
    vec(4) = twist.rot.y();
    vec(5) = twist.rot.z();

    return;
}

void unwrap_rotation(Eigen::Matrix<double, 6, 1> &x, Eigen::Matrix<double, 6, 1> &xold, std::vector<int> &counter)
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

void KDLRot2Mat(const KDL::Rotation &rot, Eigen::Matrix<double, 3, 3> &mat)
{
    //should be implemented with eigen::map
    KDL::Vector vtmp = rot.UnitX();
    mat(0, 0) = vtmp.x();
    mat(1, 0) = vtmp.y();
    mat(2, 0) = vtmp.z();
    vtmp = rot.UnitY();
    mat(0, 1) = vtmp.x();
    mat(1, 1) = vtmp.y();
    mat(2, 1) = vtmp.z();
    vtmp = rot.UnitZ();
    mat(0, 2) = vtmp.x();
    mat(1, 2) = vtmp.y();
    mat(2, 2) = vtmp.z();
}

Eigen::MatrixXd compute_JacInvPrd(const KDL::Chain &chain, const Eigen::MatrixXd &Jac, const Eigen::Matrix<double, 6, 1> &x)
{
    if (Jac.cols() != chain.getNrOfJoints())
        throw std::length_error("invalid matrix shape");
    return compute_JacInvPrd(Jac, x);
}

Eigen::MatrixXd compute_JacInvPrd(const KDL::Chain &chain, const Eigen::MatrixXd &Jac, const Eigen::Matrix<double, 6, 1> &x, const double &lambda)
{
    // add damped least squares method nice refs in: https://groups.csail.mit.edu/drl/journal_club/papers/033005/buss-2004.pdf
    if (Jac.cols() != chain.getNrOfJoints())
        throw std::length_error("invalid matrix shape");
    if (lambda > 0.0)
    {
        Eigen::MatrixXd I;
        I.setIdentity(Jac.rows(), Jac.cols());
        // Row operations returned NaN often enough
        // Eigen::MatrixXd f = (Jac*Jac.transpose() + lambda*lambda*I).colPivHouseholderQr().solve(x);
        // return Jac.transpose()*f;
        return Jac.transpose() * (Jac * Jac.transpose() + lambda * lambda * I).inverse() * x;
    }
    else
        return compute_JacInvPrd(Jac, x);
}

Eigen::MatrixXd compute_JacInvPrd(const Eigen::MatrixXd &Jac, const Eigen::Matrix<double, 6, 1> &x)
{
    if (Jac.cols() == Jac.rows())
        return Jac.colPivHouseholderQr().solve(x);
    else
        return Jac.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(x);
}

Eigen::Matrix<double, 3, 3> E(const Eigen::Quaterniond &q)
{
    Eigen::Matrix<double, 3, 3> I = I.setIdentity(3, 3);
    return q.w() * I + S(q.vec());
}

template <class T>
Eigen::Matrix<double, 3, 3> S(const T &w)
{
    /* S(w)
		 * mS=S(state_frame.M.w); //kdl
		 * mS=S(dx.segment<3>(3)); //eigen
	 */
    Eigen::Matrix<double, 3, 3> S = S.setZero(3,3);
    S(0, 1) = -w(2);
    S(0, 2) = w(1);
    S(1, 0) = w(2);
    S(1, 2) = -w(0);
    S(2, 0) = -w(1);
    S(2, 1) = w(0);

    return S;
}

} // namespace rct