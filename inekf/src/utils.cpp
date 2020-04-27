#include <utils.hpp>

using namespace Eigen;

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
    return Matrix3d::Identity();
}