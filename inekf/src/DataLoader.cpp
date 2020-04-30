// clang-format off
#include <DataLoader.hpp>
#include <iostream>
#include <cstdio>
#include <fstream>
#include <stdexcept>

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

    next_ts_ = first_ts_ = data_.begin()->first;
}

DataLoader::OutDataType DataLoader::next() {
    auto next_data = data_.equal_range(next_ts_);
    if (next_data.second == data_.end()){
        done_ = true;    
    } else {
        next_ts_ = data_.upper_bound(next_ts_)->first;
    }
    return next_data;
}

void DataLoader::parse_gps(string fn) {

    ifstream gps_in(fn);
    if(!gps_in.is_open()) throw std::runtime_error("Could not open gps file");

    string line;
    getline(gps_in, line);

    Vector3d prev_gps;

    size_t line_num = 0;
    while (getline(gps_in, line)) {
        unsigned long long ts;
        double lo, la, a;
        string str_spec {"%llu, %*u, %lf, %lf, %lf, %*s"};
        
        sscanf(line.c_str(), str_spec.c_str(), &ts, &la, &lo, &a);

        Timestamp ti {std::chrono::microseconds(ts)};
        Vector3d gps {la, lo, a};
       
        if (gps != prev_gps) {
            data_.insert(make_pair(ti, Data{ti, DataType::gps, gps}));
            prev_gps = gps;
        }
        ++line_num;
    }

    std::cout << "Read " << line_num << " lines from " << fn << std::endl;

    gps_in.close();
}

void DataLoader::parse_raw(string fn) {

    ifstream imu_in(fn);

    if(!imu_in.is_open()) throw std::runtime_error("Could not open imu file");

    string line;
    getline(imu_in, line);

    size_t line_num = 0;
    while (getline(imu_in, line)) {
        unsigned long long ts;
        double ox, oy, oz, ax, ay, az, ax_b, ay_b, az_b;
        string str_spec {"%llu, %lf, %lf, %lf, %lf, %lf, %lf, %*f, %*f, %*f, %lf, %lf, %lf, %*s"};
        
        sscanf(line.c_str(), str_spec.c_str(),
            &ts, &ox, &oy, &oz, &ax, &ay, &az, &ax_b, &ay_b, &az_b);

        Timestamp ti {std::chrono::microseconds(ts)};
        Vector3d omega {ox, oy, oz};
        Vector3d accel {ax, ay, az};
        Vector3d acc_bias {ax_b, ay_b, az_b};

        data_.insert({make_pair(ti, Data{ti, DataType::omega, omega}), 
                      make_pair(ti, Data{ti, DataType::accel, accel}),
                      make_pair(ti, Data{ti, DataType::accel_bias, acc_bias})});
        ++line_num;
    }

    std::cout << "Read " << line_num << " lines from " << fn << std::endl;

    imu_in.close();
}
