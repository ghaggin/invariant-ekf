#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_example
#include <boost/test/unit_test.hpp>

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