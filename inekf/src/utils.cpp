#include <cmath>
#include <utils.hpp>

using namespace Eigen;

using Matrix5d = Matrix<double, 5, 5>;
using Vector9d = Matrix<double, 9, 1>;

/*****************************************************************************/
// clang-format off
Matrix3d skew(const Vector3d& u) {
    return (Matrix3d() << 
            0, -u(2),  u(1), 
         u(2),     0, -u(0), 
        -u(1),  u(0),     0).finished();
}
// clang-format on

/*****************************************************************************/
Matrix3d gamma0(const Vector3d& phi)
{
    const double norm_phi = phi.norm();
    if (norm_phi > 1e-8)
    {
        return Matrix3d::Identity() + sin(norm_phi) / norm_phi * skew(phi) +
               (1 - cos(norm_phi)) / (norm_phi * norm_phi) * skew(phi) *
                   skew(phi);
    }
    return Matrix3d::Identity();
}

/*****************************************************************************/
Matrix3d gamma1(const Vector3d& phi)
{
    const double norm_phi = phi.norm();
    if (norm_phi > 1e-8)
    {
        return Matrix3d::Identity() +
               (1 - cos(norm_phi)) / pow(norm_phi, 2) * skew(phi) +
               (norm_phi - sin(norm_phi)) / pow(norm_phi, 3) * skew(phi) *
                   skew(phi);
    }
    return Matrix3d::Identity();
}

/*****************************************************************************/
Matrix3d gamma2(const Vector3d& phi)
{
    const double norm_phi = phi.norm();
    if (norm_phi > 1e-8)
    {
        return 0.5 * Matrix3d::Identity() +
               (norm_phi - sin(norm_phi)) / pow(norm_phi, 3) * skew(phi) +
               (pow(norm_phi, 2) + 2 * cos(norm_phi) - 2) /
                   (2 * pow(norm_phi, 4)) * skew(phi) * skew(phi);
    }
    return 0.5 * Matrix3d::Identity();
}

/*****************************************************************************/
Matrix5d makeTwist(const Vector9d& u)
{
    Matrix5d twist = Matrix5d::Zero();
    twist.block<3, 3>(0, 0) = skew(u.block<3, 1>(0, 0));
    twist.block<3, 1>(0, 3) = u.block<3, 1>(3, 0);
    twist.block<3, 1>(0, 4) = u.block<3, 1>(6, 0);
    return twist;
}

Eigen::Vector3d lla_to_ecef(Eigen::Vector3d lla)
{
    static const double R_earth = 6.371e6;  // m
    const double lat = lla(0) * M_PI / 180;
    const double lon = lla(1) * M_PI / 180;
    const double alt = lla(2);

    const double r = R_earth + alt;

    const double z = r * sin(lat);
    const double q = r * cos(lat);
    const double x = q * cos(lon);
    const double y = q * sin(lon);

    return (Vector3d() << x, y, z).finished();
}

Eigen::Vector3d lla_to_enu(Eigen::Vector3d lla, Eigen::Vector3d origin_lla)
{
    Vector3d origin_ecef = lla_to_ecef(origin_lla);
    Vector3d point_ecef = lla_to_ecef(lla);
    Vector3d r_ecef = point_ecef - origin_ecef;

    double phi = origin_lla(0) * M_PI / 180;
    double lam = origin_lla(1) * M_PI / 180;

    // clang-format off
    Matrix3d R = (Matrix3d() << 
        -sin(lam), cos(lam), 0, 
        -cos(lam) * sin(phi), -sin(lam) * sin(phi), cos(phi), 
        cos(lam) * cos(phi), sin(lam) * cos(phi), sin(phi)).finished();
    // clang-format on

    return R * r_ecef;
}