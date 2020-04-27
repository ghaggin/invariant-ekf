#pragma once

#include <eigen3/Eigen/Dense>

// Arange matrix for testing
// equivalent to np.arange(n*n).reshape(n,n)
template <typename Type, int n>
Eigen::Matrix<Type, n, n> arange_square(Type start_num = 0)
{
    Eigen::Matrix<double, n, n> m_out;
    for (int i = 0; i < n * n; ++i)
    {
        // indexing in collumn order
        m_out(i) = start_num++;
    }
    // return in row order
    return m_out.transpose();
}

/**
 * Gamma functions for prediction
 *  - gamma0 = matrix exponential
 *  - gamma1 = left jacobian
 *  - gamma2 = ? integral of left jacobian ?
 */

Eigen::Matrix3d skew(const Eigen::Vector3d& u);
Eigen::Matrix3d gamma0(const Eigen::Vector3d& phi);
Eigen::Matrix3d gamma1(const Eigen::Vector3d& phi);
Eigen::Matrix3d gamma2(const Eigen::Vector3d& phi);

Eigen::Matrix<double, 5, 5> makeTwist(const Eigen::Matrix<double, 9, 1>& u);
