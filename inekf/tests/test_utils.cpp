#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_example
#include <boost/test/unit_test.hpp>

#include <iostream>
#include <utils.hpp>

using namespace Eigen;

template <typename EigenMatd>
static double max_diff(const EigenMatd& m1, const EigenMatd& m2)
{
    return (m1 - m2).cwiseAbs().maxCoeff();
}

double tol = 1e-5;

BOOST_AUTO_TEST_CASE(test_arange)
{
    auto m = arange_square<double, 3>();

    BOOST_CHECK_CLOSE(m(0, 2), 2, tol);
    BOOST_CHECK_CLOSE(m(2, 0), 6, tol);
    BOOST_CHECK_CLOSE(m(2, 2), 8, tol);
}

BOOST_AUTO_TEST_CASE(test_skew)
{
    auto u = (Vector3d() << 1, 2, 3).finished();
    auto m = skew(u);
    Matrix3d nmt = -m.transpose();

    BOOST_CHECK(max_diff(m, nmt) < 1e-8);
}

// If these pass both ecef and enu conversions work
// enu sol generated from matlab scripts
BOOST_AUTO_TEST_CASE(gps_conversion)
{
    // std::cout << "hello!!!\n";
    Vector3d gps;
    Vector3d origin_gps;
    Vector3d enu;
    Vector3d enu_sol;

    origin_gps << 40, 10, 1;

    gps = origin_gps;

    enu = lla_to_enu(gps, origin_gps);
    enu_sol = Vector3d::Zero();

    BOOST_CHECK(max_diff(enu, enu_sol) < 1e-8);

    gps(0) = 41;
    gps(1) = 11;
    gps(2) = 1111;

    enu_sol(0) = 0.839302494343232e5;
    enu_sol(1) = 1.116794796570489e5;
    enu_sol(2) = -0.004215912354852e5;
    enu = lla_to_enu(gps, origin_gps);

    BOOST_CHECK(max_diff(enu, enu_sol) < 1e-8);
}