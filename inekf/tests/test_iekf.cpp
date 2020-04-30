#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_iekf
#include <boost/test/unit_test.hpp>

#include <IEKF.hpp>
#include <utils.hpp>

#include <eigen3/Eigen/Dense>

#include <iostream>
#include <tuple>

using namespace Eigen;
using namespace iekf;
using namespace std::chrono;

using Matrix5d = IEKF::Matrix5d;
using Matrix15d = IEKF::Matrix15d;
using Timestamp = IEKF::Timestamp;
using Seconds = IEKF::Seconds;

const double tol = 1e-5;

/*****************************************************************************/
// return absolute value of the max difference of any single element in two
// matrices
template <typename EigenMatd>
static double max_diff(const EigenMatd& m1, const EigenMatd& m2)
{
    return (m1 - m2).cwiseAbs().maxCoeff();
}

/*****************************************************************************/
// Defind some shorthand identity matrices
const Matrix3d Id3 = Matrix3d::Identity();
const Matrix5d Id5 = Matrix5d::Identity();
const Matrix15d Id15 = Matrix15d::Identity();

/*****************************************************************************/
BOOST_AUTO_TEST_CASE(default_constructor)
{
    IEKF iekf;
    Matrix5d mu;
    Matrix15d Sigma;
    Timestamp time;
    std::tie(mu, Sigma, time) = iekf.getState();

    auto a = (mu - Matrix5d::Identity()).cwiseAbs().maxCoeff();
    BOOST_CHECK_CLOSE(a, 0, tol);

    a = (Sigma - Matrix15d::Identity()).cwiseAbs().maxCoeff();
    BOOST_CHECK_CLOSE(a, 0, tol);
}

/*****************************************************************************/
BOOST_AUTO_TEST_CASE(constructor)
{
    auto mu_in = arange_square<double, 5>();
    auto Sigma_in = arange_square<double, 15>();

    IEKF iekf(mu_in, Sigma_in);

    Matrix5d mu_out;
    Matrix15d Sigma_out;
    Timestamp time_out;
    std::tie(mu_out, Sigma_out, time_out) = iekf.getState();

    auto a = (mu_out - arange_square<double, 5>()).cwiseAbs().maxCoeff();
    BOOST_CHECK_CLOSE(a, 0, tol);

    a = (Sigma_out - arange_square<double, 15>()).cwiseAbs().maxCoeff();
    BOOST_CHECK_CLOSE(a, 0, tol);
}

/*****************************************************************************/
BOOST_AUTO_TEST_CASE(interfaces)
{
    auto mu = arange_square<double, 5>();
    auto Sigma = Matrix15d::Identity();
    IEKF iekf(mu, Sigma);
    auto R = iekf.R();
    auto p = iekf.p();
    auto v = iekf.v();

    auto R_sol = (Matrix3d() << 0, 1, 2, 5, 6, 7, 10, 11, 12).finished();
    auto p_sol = (Vector3d() << 4, 9, 14).finished();
    auto v_sol = (Vector3d() << 3, 8, 13).finished();

    auto a = (R - R_sol).cwiseAbs().maxCoeff();
    BOOST_CHECK_CLOSE(a, 0, tol);

    a = (p - p_sol).cwiseAbs().maxCoeff();
    BOOST_CHECK_CLOSE(a, 0, tol);

    a = (v - v_sol).cwiseAbs().maxCoeff();
    BOOST_CHECK_CLOSE(a, 0, tol);
}

/*****************************************************************************/
// Run a test with zero inputs for accel and gyro, should return
// the starting state
BOOST_AUTO_TEST_CASE(prediction_zero_meas)
{
    // Default construct with mu = Id
    IEKF iekf{};

    auto acc = Vector3d::Zero();
    auto gyro = Vector3d::Zero();
    Timestamp time{Seconds{0}};

    // Add imu measurement
    iekf.addImu(time, acc, gyro);

    auto mu = iekf.mu();

    auto a = max_diff(mu, Id5);
    BOOST_CHECK_CLOSE(a, 0, tol);
}

/*****************************************************************************/
// Test with constant gyroscope measurements and accel just measureing gravity
BOOST_AUTO_TEST_CASE(prediction_const_gyro)
{
    // Default construct with mu = Id
    IEKF iekf;

    auto g = iekf.g();

    auto acc = (Vector3d() << 0.0, 0.0, g).finished();
    auto gyro = (Vector3d() << 2 * M_PI, 0, 0).finished();

    Timestamp time{Seconds{1}};

    iekf.addImu(time, acc, gyro);

    auto R = iekf.R();
    auto R_sol = Id3;

    auto a = max_diff(R, R_sol);
    BOOST_CHECK(a < 1e-8);
}

/*****************************************************************************/
// Test with constant accel but no gyro input.  Body aligned with world frame
// should simulate pure rectilinear motion
BOOST_AUTO_TEST_CASE(prediction_const_acc)
{
    // Default construct with mu = Id
    IEKF iekf;

    auto g = iekf.g();

    // Measure 1 m/s^2 accel in body frame x
    // keep body frame aligned with world frame, look for accel in x
    auto acc = (Vector3d() << 1.0, 0.0, g).finished();
    auto gyro = Vector3d::Zero();

    // Default constructor sets time to zero
    // Set first imu measurement at 1 seconds
    Timestamp time{Seconds{1}};

    // Add imu measurement
    // Accelerate 1 m/s^2 in x for 1 sec
    iekf.addImu(time, acc, gyro);

    // Extract state
    auto R = iekf.R();
    auto p = iekf.p();
    auto v = iekf.v();

    // Set solution
    //  R - vehicle should not have rotated -> identity
    //  p - should move 1/2*a*t^2, since t = 1 and a = 1, p = 1/2
    //  v - should change to a*t, since a = 1 and t = 1, v = 1
    auto R_sol = Id3;
    auto p_sol = (Vector3d{} << 0.5, 0, 0).finished();
    auto v_sol = (Vector3d{} << 1., 0, 0).finished();

    // Check the state vs solution
    auto a = max_diff(R, R_sol);
    BOOST_CHECK(a < 1e-8);

    a = max_diff(p, p_sol);
    BOOST_CHECK(a < 1e-8);

    a = max_diff(v, v_sol);
    BOOST_CHECK(a < 1e-8);

    // Now accelerate in y for 1 sec
    // note that velocity in x is now 1
    acc = (Vector3d{} << 0, 1, g).finished();
    time = Timestamp{Seconds{2}};
    iekf.addImu(time, acc, gyro);
    R = iekf.R();
    p = iekf.p();
    v = iekf.v();
    p_sol = (Vector3d{} << 1.5, 0.5, 0).finished();
    v_sol = (Vector3d{} << 1., 1., 0).finished();
    BOOST_CHECK(max_diff(R, R_sol) < 1e-8);
    BOOST_CHECK(max_diff(p, p_sol) < 1e-8);
    BOOST_CHECK(max_diff(v, v_sol) < 1e-8);

    // Now accelerate in z for 1 sec
    // note that x and y velocty are now 1
    acc = (Vector3d{} << 0, 0, g + 1).finished();
    time = Timestamp{Seconds{3}};
    iekf.addImu(time, acc, gyro);
    R = iekf.R();
    p = iekf.p();
    v = iekf.v();
    p_sol = (Vector3d{} << 2.5, 1.5, 0.5).finished();
    v_sol = (Vector3d{} << 1., 1., 1.).finished();
    BOOST_CHECK(max_diff(R, R_sol) < 1e-8);
    BOOST_CHECK(max_diff(p, p_sol) < 1e-8);
    BOOST_CHECK(max_diff(v, v_sol) < 1e-8);
}

/*****************************************************************************/
// Test with constant accel and gyro over 1 sec
// Correct output determined from the prediction function
// in scripts/LIEKF.m with default initial conditions and
//  w = [0.3, 0.5, 0.7]
//  a = [0.11, 0.13, 0.17]
//  dt = 0.19
BOOST_AUTO_TEST_CASE(test_accel_and_gyro)
{
    // Solution determined from scripts/LIEKF.m
    Matrix5d mu_sol =
        (Matrix5d{} << 0.986676318023551, -0.129636068151535, 0.098307340955289,
            0.020762064018513, 0.001977230017723, 0.135037560844690,
            0.989557114126567, -0.050414036166701, 0.025168814239892,
            0.002376210384301, -0.090745251184872, 0.063017519117396,
            0.993878308281091, -1.831875751893571, -0.174019677424953, 0, 0, 0,
            1.000000000000000, 0, 0, 0, 0, 0, 1.000000000000000)
            .finished();

    Vector3d w{0.3, 0.5, 0.7};
    Vector3d a{0.11, 0.13, 0.17};
    Timestamp time{Seconds{0.19}};

    IEKF iekf{};

    iekf.addImu(time, a, w);

    Matrix5d mu = iekf.mu();

    BOOST_CHECK(max_diff(mu, mu_sol) < 1e-8);
}

/*****************************************************************************/
// Simple test, zero update should keep filter at origin
BOOST_AUTO_TEST_CASE(test_update)
{
    IEKF iekf{};
    Vector3d gps{0, 0, 0};       // equator, prime mer, msl
    Timestamp time{Seconds{0}};  // doesnt matter for gps meas

    iekf.addGps(time, gps);

    Matrix5d mu = iekf.mu();
    BOOST_CHECK(max_diff(mu, Id5) < 1e-8);
}

/*****************************************************************************/
// Now give the filter a nonzero update, solution generated from matlab version
// of the filter
BOOST_AUTO_TEST_CASE(test_update2)
{
    Matrix5d mu_sol;
    mu_sol << 1, 0, 0, 0, 1.111554670002041e5, 0, 1, 0, 0, 0.555946668845763e5,
        0, 0, 1, 0, -0.024238932013875e5, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;

    IEKF iekf{};
    Vector3d gps{1, 2, 3};
    Timestamp time{Seconds{0}};  // doesnt matter for gps meas

    // Set filter origin
    // without this origin is set to first update measurement
    iekf.resetFilter(time, Vector3d::Zero());

    iekf.addGps(time, gps);

    Matrix5d mu = iekf.mu();

    BOOST_CHECK(max_diff(mu, mu_sol) < 1e-6);  // big numbers
}

BOOST_AUTO_TEST_CASE(test_two_predictions)
{
    IEKF iekf{};
    Vector3d w{0.3, 0.5, 0.7};
    Vector3d a{0.11, 0.13, 0.17};
    Timestamp t1{Seconds{0.19}};
    iekf.addImu(t1, a, w);

    // Next time stamp (same meas)
    Timestamp t2{Seconds{0.27}};
    iekf.addImu(t2, a, w);

    Matrix5d mu = iekf.mu();
    Matrix5d mu_sol;
    mu_sol << 0.973162730597739, -0.181659800386756, 0.141258687162937,
        0.029396711070180, 0.003984086586539, 0.192539774468753,
        0.978965383441471, -0.067492320087659, 0.036044893440882,
        0.004823667922829, -0.126026723448141, 0.092878926278987,
        0.987669362707069, -2.603344942916421, -0.351427942767680, 0, 0, 0,
        1.000000000000000, 0, 0, 0, 0, 0, 1.000000000000000;

    BOOST_CHECK(max_diff(mu, mu_sol) < 1e-8);
}

BOOST_AUTO_TEST_CASE(test_predict_then_correct)
{
    IEKF iekf{};
    Vector3d w{0.3, 0.5, 0.7};
    Vector3d a{0.11, 0.13, 0.17};
    Timestamp t1{Seconds{0.19}};
    iekf.addImu(t1, a, w);

    // Next time stamp (same meas)
    Timestamp t2{Seconds{0.27}};
    Vector3d gps{0, 0, 0};
    iekf.addGps(t2, gps);

    Matrix5d mu = iekf.mu();
    Matrix5d mu_sol;
    mu_sol << 0.986693433490178, -0.129647882481378, 0.098119798590595,
        0.020555963014757, 0.000876004775617, 0.135058037659745,
        0.989542800730808, -0.050639628605745, 0.024911962997068,
        0.001052764543022, -0.090528419686102, 0.063217656472885,
        0.993885372233243, -1.813931200473841, -0.077918649513698, 0, 0, 0,
        1.000000000000000, 0, 0, 0, 0, 0, 1.000000000000000;

    BOOST_CHECK(max_diff(mu, mu_sol) < 1e-8);
}

BOOST_AUTO_TEST_CASE(test_correct_then_predict)
{
    IEKF iekf{};

    Timestamp t1{Seconds{0.19}};
    Vector3d gps{0, 0, 0};
    iekf.addGps(t1, gps);

    Vector3d w{0.3, 0.5, 0.7};
    Vector3d a{0.11, 0.13, 0.17};
    Timestamp t2{Seconds{0.27}};
    iekf.addImu(t2, a, w);

    Matrix5d mu = iekf.mu();
    Matrix5d mu_sol;
    mu_sol << 0.973162730597739, -0.181659800386756, 0.141258687162937,
        0.029396711070180, 0.003984086586539, 0.192539774468753,
        0.978965383441471, -0.067492320087659, 0.036044893440882,
        0.004823667922829, -0.126026723448141, 0.092878926278987,
        0.987669362707069, -2.603344942916421, -0.351427942767680, 0, 0, 0,
        1.000000000000000, 0, 0, 0, 0, 0, 1.000000000000000;

    BOOST_CHECK(max_diff(mu, mu_sol) < 1e-8);
}

BOOST_AUTO_TEST_CASE(test_predict_correct2)
{
    IEKF iekf{};
    Vector3d w{0.3, 0.5, 0.7};
    Vector3d a{0.11, 0.13, 0.17};
    Timestamp t1{Seconds{0.19}};
    iekf.addImu(t1, a, w);

    // Next time stamp (same meas)
    Timestamp t2{Seconds{0.27}};
    Vector3d gps{0, 0, 0};
    iekf.addGps(t2, gps);

    Matrix5d mu = iekf.mu();
    Matrix5d mu_sol;
    mu_sol << 0.986693433490178, -0.129647882481378, 0.098119798590595,
        0.020555963014757, 0.000876004775617, 0.135058037659745,
        0.989542800730808, -0.050639628605745, 0.024911962997068,
        0.001052764543022, -0.090528419686102, 0.063217656472885,
        0.993885372233243, -1.813931200473841, -0.077918649513698, 0, 0, 0,
        1.000000000000000, 0, 0, 0, 0, 0, 1.000000000000000;

    BOOST_CHECK(max_diff(mu, mu_sol) < 1e-8);
}

/*****************************************************************************/
BOOST_AUTO_TEST_CASE(test_predict_correct3)
{
    IEKF iekf{};
    Vector3d w{0.3, 0.5, 0.7};
    Vector3d a{0.11, 0.13, 0.17};
    Timestamp t1{Seconds{0.19}};
    iekf.addImu(t1, a, w);

    // Next time stamp (same meas)
    Timestamp t2{Seconds{0.27}};
    iekf.addImu(t2, a, w);

    // Add correction
    Vector3d gps = Vector3d::Zero();
    Timestamp t3{Seconds{0.53}};
    iekf.addGps(t3, gps);

    Matrix5d mu = iekf.mu();
    Matrix5d mu_sol;  // solution from matlab
    mu_sol << 0.973255763823231, -0.181727681256770, 0.140528531088517,
        0.028814444001715, 0.001612373862480, 0.192651428928085,
        0.978881893938724, -0.068378831891927, 0.035268446408571,
        0.001951947862742, -0.125134508097588, 0.093623114581685,
        0.987712542847968, -2.552727205507384, -0.148982331045971, 0, 0, 0,
        1.000000000000000, 0, 0, 0, 0, 0, 1.000000000000000;

    std::cout << max_diff(mu, mu_sol) << std::endl;

    BOOST_CHECK(max_diff(mu, mu_sol) < 1e-8);

    std::cout << mu << std::endl;
    std::cout << mu_sol << std::endl;
}

/*****************************************************************************/
BOOST_AUTO_TEST_CASE(test_predict_correct4)
{
    IEKF iekf{};
    Vector3d w{0.3, 0.5, 0.7};
    Vector3d a{0.11, 0.13, 0.17};
    Timestamp t1{Seconds{0.19}};
    iekf.addImu(t1, a, w);

    // Next time stamp (same meas)
    Timestamp t2{Seconds{0.27}};
    iekf.addImu(t2, a, w);

    Timestamp t4{Seconds{0.57}};
    iekf.addImu(t4, a, w);

    // Add correction
    Vector3d gps = Vector3d::Zero();
    Timestamp t5{Seconds{0.83}};
    iekf.addGps(t5, gps);

    Matrix5d mu = iekf.mu();
    Matrix5d mu_sol;  // solution from matlab
    mu_sol << 0.885544010270727, -0.360280157014254, 0.293274639775456,
        0.054586397620639, -0.001163044332474, 0.408845094508917,
        0.904173748861653, -0.123755891033354, 0.065149336186669,
        -0.001654357353935, -0.220584438638890, 0.229495185856368,
        0.947984422392645, -5.022384894436503, -0.517530792432281, 0, 0, 0,
        1.000000000000000, 0, 0, 0, 0, 0, 1.000000000000000;

    std::cout << max_diff(mu, mu_sol) << std::endl;

    BOOST_CHECK(max_diff(mu, mu_sol) < 1e-8);

    std::cout << mu << std::endl;
    std::cout << mu_sol << std::endl;
}