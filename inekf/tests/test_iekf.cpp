#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_iekf
#include <boost/test/unit_test.hpp>

#include <IEKF.hpp>

#include <eigen3/Eigen/Dense>

#include <iostream>
#include <tuple>

using namespace Eigen;
using namespace iekf;

BOOST_AUTO_TEST_CASE(constructor)
{
    IEKF iekf;
    IEKF::Matrix5d mu;
    IEKF::Matrix9d Sigma;
    IEKF::Timestamp time;
    std::tie(mu, Sigma, time) = iekf.getState();

    auto a = (mu - Matrix<double, 5, 5>::Identity()).cwiseAbs().maxCoeff();
    BOOST_CHECK_CLOSE(a, 1, 1e-5);
}