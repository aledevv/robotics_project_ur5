#include "robotics_project_ur5/kinematics.h"
#include "Eigen/Dense"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "boost/shared_ptr.hpp"
#include "std_msgs/Float64MultiArray.h"
#include <iostream>
#include <cmath>
#include <stdexcept>

using namespace Eigen;

using M4d = Matrix4d;
using M3d = Matrix3d;
using V6d = Vector6d;
using Jacobian = Matrix<double, 6, 6>;

using V8d = Vector8d;
using V6d = Vector6d;
using V3d = Vector3d;
using Qd = Quaterniond;
using M4d = Matrix4d;
using M3d = Matrix3d;
using Jacobian = Matrix<double, 6, 6>;
using Path = std::vector<std::pair<V6d, V2d>>;

M4d T10f(double th1) {
    return Transform<double, 3, Isometry>(AngleAxisd(th1, Vector3d::UnitZ())).matrix();
}

M4d T21f(double th2) {
    return Transform<double, 3, Isometry>(AngleAxisd(th2, Vector3d::UnitZ())).matrix();
}

M4d T32f(double th3) {
    return Transform<double, 3, Isometry>(AngleAxisd(th3, Vector3d::UnitZ())).matrix();
}

M4d T43f(double th4) {
    return Transform<double, 3, Isometry>(AngleAxisd(th4, Vector3d::UnitZ())).matrix();
}

M4d T54f(double th5) {
    return Transform<double, 3, Isometry>(AngleAxisd(th5, Vector3d::UnitZ())).matrix();
}

M4d T65f(double th6) {
    return Transform<double, 3, Isometry>(AngleAxisd(th6, Vector3d::UnitZ())).matrix();
}

M3d rotation_matrix_z_axis(double alpha) {
    return AngleAxisd(alpha, Vector3d::UnitZ()).toRotationMatrix();
}

M4d direct_kin(V6d js) {
    return T10f(js(0)) * T21f(js(1)) * T32f(js(2)) * T43f(js(3)) * T54f(js(4)) * T65f(js(5));
}

Jacobian jacobian(V6d js) {
    Jacobian J;
    J.setZero();

    double c1 = cos(js(0)), s1 = sin(js(0));
    double c2 = cos(js(1)), s2 = sin(js(1));
    double c3 = cos(js(2)), s3 = sin(js(2));
    double c4 = cos(js(3)), s4 = sin(js(3));
    double c5 = cos(js(4)), s5 = sin(js(4));
    double c6 = cos(js(5)), s6 = sin(js(5));

    J << d(4) * (c1 * c4 + (c1 * c2 * c3 - s1 * s3) * s4) + d(2) * c1 + d(3) * c1 - a(2) * (c2 * c3 * s1 + c1 * s2 * s3) - a(1) * (c1 * c2 * s3 + c1 * s2 * c3) - d(4) * (s1 * s2 * s3 + c1 * c3) * s4,
         d(4) * (c4 * s1 - (c1 * c2 * c3 + s1 * s3) * s4) + d(2) * s1 + d(3) * s1 + a(2) * (c2 * c3 * c1 - s1 * s2 * s3) + a(1) * (c1 * c3 - c2 * s1 * s3) + d(4) * (c1 * s2 * s3 - c2 * c3 * s1) * s4,
         0, 0, 0, 1,
         -c1 * (a(2) * s2 * s3 + a(1) * s2 + d(4) * (s2 * s3 * s4 - c2 * c3 * c4) - d(4) * c4 * s5 * (c2 * s3 - c3 * s2)) + s1 * (a(2) * c3 - d(4) * s3 - a(1) * c2 - d(4) * (c2 * s3 + c3 * s2)) + d(4) * c4 * c5 * (c2 * c3 * s4 + c4 * s2),
         -s1 * (a(2) * s2 * s3 + a(1) * s2 + d(4) * (s2 * s3 * s4 - c2 * c3 * c4) - d(4) * c4 * s5 * (c2 * s3 - c3 * s2)) - c1 * (a(2) * c3 - d(4) * s3 - a(1) * c2 - d(4) * (c2 * s3 + c3 * s2)) - d(4) * c4 * c5 * (c2 * c3 * s4 + c4 * s2),
         a(2) * c2 * c3 - (d(4) * (s2 * s3 * s4 - c2 * c3 * c4 + c4 * s2) - d(4) * c4 * c5 * (s2 * s3 * s4 - c2 * c3 * c4 + c4 * s2)) / 2 + a(1) * c2 + d(4) * (c2 * s3 + c3 * s2) + d(4) * c4 * c5 * (c2 * c3 * s4 + c4 * s2) / 2 - d(4) * c4 * s5 * (c2 * c3 * c4 + s2 * s3) / 2 + d(4) * c4 * s5 * (c4 * s2 - c2 * c3 * s4) / 2 + d(4) * c4 * c5 * (c2 * c3 * s4 - c4 * s2) / 2 - d(4) * c4 * s5 * (c2 * c3 * s4 - c4 * s2) / 2 + d(4) * c4 * c5 * (c4 * s2 - c2 * c3 * s4) / 2 + d(4) * c4 * s5 * (c4 * s2 - c2 * c3 * s4) / 2 + d(4) * c4 * s5 * (c2 * c3 * c4 - s2 * s3) / 2 - d(4) * c4 * s5 * (c2 * c3 * c4 - s2 * s3) /
}


/**
 * @brief Linear interpolation for vectors.
 *
 * @param t Time parameter.
 * @param x1 Initial vector.
 * @param x2 Final vector.
 * @return Interpolated vector.
 */
V3d linearInterpolation(double t, const V3d& x1, const V3d& x2) {
    const double n_t = t / d_path;
    return (n_t > 1.0) ? x2 : (n_t * x2 + (1.0 - n_t) * x1);
}

/**
 * @brief Spherical linear interpolation for quaternions.
 *
 * @param t Time parameter.
 * @param q1 Initial quaternion.
 * @param q2 Final quaternion.
 * @return Interpolated quaternion.
 */
Qd slerp(double t, const Qd& q1, const Qd& q2) {
    const double n_t = t / d_path;
    return (n_t > 1.0) ? q2 : q1.slerp(n_t, q2);
}



// Function to compute linear interpolation for vectors
V3d linearInterpolation(double t, const V3d& x1, const V3d& x2) {
    const double n_t = t / d_path;
    return (n_t > 1.0) ? x2 : (n_t * x2 + (1.0 - n_t) * x1);
}

// Function to compute spherical linear interpolation for quaternions
Qd slerp(double t, const Qd& q1, const Qd& q2) {
    const double n_t = t / d_path;
    return (n_t > 1.0) ? q2 : q1.slerp(n_t, q2);
}

// Function to perform path differential inverse kinematics with quaternions
Path pathDifferentialInverseKinematics(V8d mr, V3d i_p, V3d f_p, Qd i_q, Qd f_q) {
    V2d gs {mr(6), mr(7)};
    V6d js_k, js_dot_k, fv;
    Path path;

    M4d tm_k;
    V3d p_k, av_k, pv_k, perr_k;
    M3d rm_k;

    Qd q_k, qv_k, qerr_k;

    Jacobian j_k, invj_k;

    M3d Kp = M3d::Identity() * 10;
    M3d Kq = M3d::Identity() * 1;

    for (int i = 0; i < 6; ++i) js_k(i) = mr(i);
    path.emplace_back(js_k, gs);

    for (double t = dt; t < d_path; t += dt) {
        tm_k = direct_kin(js_k);
        p_k = tm_k.block(0, 3, 3, 1);
        rm_k = tm_k.block(0, 0, 3, 3);
        q_k = Qd(rm_k);

        pv_k = (linearInterpolation(t, i_p, f_p) - linearInterpolation(t - dt, i_p, f_p)) / dt;
        qv_k = slerp(t + dt, i_q, f_q) * slerp(t, i_q, f_q).conjugate();
        av_k = (qv_k.vec() * 2) / dt;

        j_k = jacobian(js_k);
        invj_k = (j_k.transpose() * j_k + Jacobian::Identity() * 0.0001).inverse() * j_k.transpose();

        if (abs(j_k.determinant()) < 0.00001) {
            std::cerr << "Near singular configuration" << std::endl;
        }

        qerr_k = slerp(t, i_q, f_q) * q_k.conjugate();
        perr_k = linearInterpolation(t, i_p, f_p) - p_k;

        fv << pv_k + (Kp * perr_k), av_k + (Kq * qerr_k.vec());

        js_dot_k = invj_k * fv;
        js_k = js_k + (js_dot_k * dt);

        path.emplace_back(js_k, gs);
    }

    return path;
}


// Assuming Jac is a function pointer or std::function with the following signature:
// Eigen::MatrixXd Jac(const Eigen::VectorXd& q, double scaleFactor);

Eigen::VectorXd invDiffKinematiControlCompleteQuaternion(
    std::function<Eigen::MatrixXd(const Eigen::VectorXd&, double)> Jac,
    const Eigen::VectorXd& q, const Eigen::Vector3d& xe, const Eigen::Vector3d& xd,
    const Eigen::Vector3d& vd, const Eigen::Vector3d& omegad, const Eigen::Quaterniond& qe,
    const Eigen::Quaterniond& qd, const Eigen::Matrix3d& Kp, const Eigen::Matrix3d& Kq,
    double scaleFactor)
{
    Eigen::MatrixXd J = Jac(q, scaleFactor);
    if (std::abs(J.determinant()) < 1e-3)
    {
        std::cerr << "Near singular configuration" << std::endl;
    }
    Eigen::Quaterniond qp = qd * qe.conjugate();
    Eigen::Vector3d eo = qp.vec();
    std::cout << eo.norm() << std::endl;
    Eigen::VectorXd dotQ = J.inverse() * (Eigen::VectorXd(6) << vd + Kp * (xd - xe), omegad + Kq * eo).finished();
    return dotQ;
}


// Assuming direct is a function that takes qk and scaleFactor and returns xe and Re
// Assuming invDiffKinematiControlCompleteQuaternion is a function that computes control

std::vector<Eigen::VectorXd> invDiffKinematicControlSimCompleteQuaternion(
    std::function<void(const Eigen::VectorXd&, double, Eigen::VectorXd&, Eigen::Matrix3d&)> direct,
    std::function<Eigen::VectorXd(const Eigen::MatrixXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::Quaterniond&, const Eigen::Quaterniond&, double, double, double)> invDiffKinematiControlCompleteQuaternion,
    const std::function<Eigen::VectorXd(double)>& xd,
    const std::function<Eigen::Quaterniond(double)>& qd,
    const Eigen::VectorXd& TH0,
    double Kp,
    double Kphi,
    double minT,
    double maxT,
    double Dt,
    double scaleFactor)
{
    std::vector<double> T;
    for (double t = minT; t <= maxT; t += Dt) {
        T.push_back(t);
    }
    size_t L = T.size();
    std::vector<Eigen::VectorXd> q;
    Eigen::VectorXd qk = TH0;
    q.push_back(qk);

    for (size_t i = 1; i < L - 1; ++i) {
        double t = T[i];
        Eigen::VectorXd xe;
        Eigen::Matrix3d Re;
        direct(qk, scaleFactor, xe, Re);
        Eigen::Quaterniond qe(Re);
        Eigen::VectorXd vd = (xd(t) - xd(t - Dt)) / Dt;
        Eigen::Quaterniond work = (Eigen::Quaterniond(1, 0, 0, 0).slerp(2.0 / Dt, qd(t + Dt) * qd(t).conjugate()));
        Eigen::Vector3d omegad(work.x(), work.y(), work.z());
        Eigen::VectorXd dotqk = invDiffKinematiControlCompleteQuaternion(Jac, qk, xe, xd(t), vd, omegad, qe, qd(t), Kp, Kphi, scaleFactor);
        Eigen::VectorXd qk1 = qk + dotqk * Dt;
        q.push_back(qk1);
        qk = qk1;
    }

    return q;
}
