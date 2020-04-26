#pragma once

#include <eigen3/Eigen/Dense>

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