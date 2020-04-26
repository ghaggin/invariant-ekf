#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_sophus
#include <boost/test/unit_test.hpp>

#include <eigen3/Eigen/Dense>
#include <sophus/so3.hpp>

#include <iostream>

BOOST_AUTO_TEST_CASE(test_exp_log)
{
    // Create a vector in the tangent space
    // using eigen
    Eigen::Vector3d omega(0.1, 0.2, 0.3);

    // Calculate the exponential map of omega which
    // maps elements from the tangent space to the lie
    // group
    Sophus::SO3d R = Sophus::SO3d::exp(omega);

    // Calculate the log map from the lie group
    // back to the tangent space
    Eigen::Vector3d omega_out = R.log();

    // Check that the original tangent space elemtent
    // is recovered
    BOOST_CHECK_CLOSE(omega_out[0], omega[0], 0.01);
    BOOST_CHECK_CLOSE(omega_out[1], omega[1], 0.01);
    BOOST_CHECK_CLOSE(omega_out[2], omega[2], 0.01);
}