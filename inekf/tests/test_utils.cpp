#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_example
#include <boost/test/unit_test.hpp>

#include <utils.hpp>

using namespace Eigen;

double tol = 1e-5;

BOOST_AUTO_TEST_CASE(test_arange)
{
    auto m = arange_square<double, 3>();

    BOOST_CHECK_CLOSE(m(0, 2), 2, tol);
    BOOST_CHECK_CLOSE(m(2, 0), 6, tol);
    BOOST_CHECK_CLOSE(m(2, 2), 8, tol);
}