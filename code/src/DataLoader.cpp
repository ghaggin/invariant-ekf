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
using timestamp = std::chrono::time_point<std::chrono::system_clock,
        std::chrono::duration<double>>;
using Eigen::Vector3d;

//#include <boost/tokenizer.hpp>
// using boost::tokenizer;
// using boost::escaped_list_separator;

DataLoader::DataLoader(string data_dir) {
    string imu_fn {data_dir + "/Log Files/RawAccel.csv"};
    string gyro_fn {data_dir + "/Log Files/RawGyro.csv"};
    string gt_fn {data_dir + "/Log Files/GroundTruthAGL.csv"};

    parse_raw(imu_fn, imu);
    parse_raw(gyro_fn, gyro);
    parse_gps_gt(gt_fn);

    cout << imu.size() << endl;
    cout << gyro.size() << endl;
    cout << gps.size() << endl;
    cout << gt.size() << endl;

    std::for_each(gps.rbegin(), gps.rend(), [this](auto& in){
        in.second -= gps.begin()->second;
    });

    std::for_each(gt.rbegin(), gt.rend(), [this](auto& in){
        in.second -= gt.begin()->second;
    });
}

void DataLoader::parse_gps_gt(string fn){
    ifstream gps_in(fn);
    assert(gps_in.is_open());

    string temp;
    getline(gps_in, temp);

    while (getline(gps_in, temp)) {
        size_t ts;
        Vector3d gt_vec, gps_vec;
        std::tie(ts, gt_vec, gps_vec) = parse_gps_gt_line(temp);

        gt.insert(std::make_pair(ts, gt_vec));
        gps.insert(std::make_pair(ts, gps_vec));
    }

}

std::tuple<size_t, Vector3d, Vector3d> DataLoader::parse_gps_gt_line(string line) {
    unsigned long long ts;
    double gtx, gty, gtz, gpsx, gpsy, gpsz;

    sscanf(line.c_str(), "%llu,%lf,%lf,%lf,%*f,%*f,%*f,%lf,%lf,%lf,%*s",
        &ts, &gpsx, &gpsy, &gpsz, &gtx, &gty, &gtz);

    Vector3d gt {gtx, gty, gtz};
    Vector3d gps {gpsx, gpsy, gpsz};

    return std::tie(ts, gt, gps);
}

void DataLoader::parse_raw(string fn,
    std::map<Timestamp, Eigen::Vector3d> &map_data){

    ifstream imu_in(fn);
    assert(imu_in.is_open());

    string temp;
    getline(imu_in, temp);

    while (getline(imu_in, temp)) {
        map_data.insert(parse_raw_line(temp));
    }
}

std::pair<timestamp, Eigen::Vector3d> DataLoader::parse_raw_line(string line){
    unsigned long long ts;
    double x, y, z;
   
    sscanf(line.c_str(), "%llu, %*u, %lf, %lf, %lf, %*s", &ts, &x, &y, &z);

    timestamp ti {std::chrono::microseconds(ts)};
    Vector3d accel {x, y, z};

    return std::make_pair(ti, accel);
}

