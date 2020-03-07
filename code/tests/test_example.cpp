#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_example
#include <boost/test/unit_test.hpp>

#include <example.hpp>

BOOST_AUTO_TEST_CASE(test_example)
{
    Example ex;
    BOOST_CHECK_EQUAL(ex.retOne(), 1);
}