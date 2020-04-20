#include <DataLoader.hpp>

#include <iostream>
#include <cstdio>
#include <fstream>

using std::cout;
using std::endl;
using std::ifstream;
using std::getline;
using std::string;
using std::tie;
using std::make_pair;
using std::for_each;
using Eigen::Vector3d;

DataLoader::DataLoader(string data_dir) {
    string pose_fn {data_dir + "/Log Files/OnboardPose.csv"};
    string gps_fn {data_dir + "/Log Files/OnboardGPS.csv"};

    parse_raw(pose_fn);
    parse_gps(gps_fn);

    cout << data_.size() << endl;
}

void DataLoader::parse_gps(string fn) {

    ifstream gpu_in(fn);
    assert(gpu_in.is_open());

    string line;
    getline(gpu_in, line);

    Timestamp prev_ti;
    Vector3d prev_gps;

    while (getline(gpu_in, line)) {
        unsigned long long ts;
        double lo, la, a;
        string str_spec {"%llu, %*u, %lf, %lf, %lf, %*s"};
        
        sscanf(line.c_str(), str_spec.c_str(), &ts, &lo, &la, &a);

        Timestamp ti {std::chrono::microseconds(ts)};
        Vector3d gps {lo, la, a};
       
        if (prev_ti == ti) {
            assert(gps == prev_gps);
            data_.insert(make_pair(ti, Data{ti, DataType::gps, gps}));

            prev_gps = gps;
            prev_ti = ti;
        }
    }
}

void DataLoader::parse_raw(string fn) {

    ifstream imu_in(fn);
    assert(imu_in.is_open());

    string line;
    getline(imu_in, line);

    while (getline(imu_in, line)) {
        unsigned long long ts;
        double ox, oy, oz, ax, ay, az, ax_b, ay_b, az_b;
        string str_spec {"%llu, %lf, %lf, %lf, %lf, %lf, %lf, %*f, %*f, %*f, %*s"};
        
        sscanf(line.c_str(), str_spec.c_str(),
            &ox, &oy, &oz, &ax, &ay, &az, &ax_b, &ay_b, &az_b);

        Timestamp ti {std::chrono::microseconds(ts)};
        Vector3d omega {ox, oy, oz};
        Vector3d accel {ax, ay, az};
        Vector3d acc_bias {ax_b, ay_b, az_b};

        data_.insert(make_pair(ti, Data{ti, DataType::omega, omega}));
        data_.insert(make_pair(ti, Data{ti, DataType::accel, accel}));
        data_.insert(make_pair(ti, Data{ti, DataType::accel_bias, acc_bias}));
    }
}
