#include <utils.h>

namespace rct
{
void convert_jarray(const KDL::Chain chain, const Eigen::MatrixXd &con, KDL::JntArray &var)
{
    if (con.rows() != chain.getNrOfJoints() || con.cols() != 1)
        throw std::length_error("invalid matrix shape");
    var.data = con;
}

void convert_jarray(const KDL::Chain chain, const KDL::JntArray &con, Eigen::MatrixXd &var)
{
    if (var.rows() != chain.getNrOfJoints() || var.cols() != 1)
        throw std::length_error("invalid matrix shape");
    var = con.data;
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

    //state_frame.M.R.GetRPY(x(3), x(4), x(5));
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
} // namespace rct