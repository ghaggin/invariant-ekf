#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_example
#include <boost/test/unit_test.hpp>

#include <Eigen/Dense>
#include <example.hpp>
#include <iostream>

using namespace Eigen;

BOOST_AUTO_TEST_CASE(test_example)
{
    Example ex;
    BOOST_CHECK_EQUAL(ex.retOne(), 1);
}

BOOST_AUTO_TEST_CASE(test_eigen_zeros)
{
    auto z3 = Matrix3d::Zero();

    std::cout << z3;

    for (int i = 0; i < 9; ++i)
    {
        BOOST_CHECK(z3(i) < 1e-8);
    }
}