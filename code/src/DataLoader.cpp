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


    imu = parse_imu(imu_fn);
    gyro = parse_imu(gyro_fn);
    gps = parse_gps(gt_fn);

    cout << imu.size() << endl;
    cout << gyro.size() << endl;
    cout << gps.size() << endl;
}

std::map<timestamp, Vector3d> DataLoader::parse_gps(string fn){
    ifstream gps_in(fn);
    assert(gps_in.is_open());

    string temp;
    getline(gps_in, temp);

    std::map<timestamp, Vector3d> umap;
    
    while (getline(gps_in, temp)) {
        umap.insert(parse_gps_line(temp));
    }

    return umap;
}

std::pair<timestamp, Eigen::Vector3d> DataLoader::parse_gps_line(string line){
    unsigned long long ts;
    double lat, lon, alt;
   
    sscanf(line.c_str(), "%llu, %*u, %lf, %lf, %lf, %*s", &ts, &lat, &lon, &alt);

    timestamp ti {std::chrono::microseconds(ts)};
    Vector3d gps {lat, lon, alt};

    return std::make_pair(ti, gps);
}

std::map<timestamp, Vector3d> DataLoader::parse_imu(string fn){
    ifstream imu_in(fn);
    assert(imu_in.is_open());

    string temp;
    getline(imu_in, temp);

    std::map<timestamp, Vector3d> umap;
    
    while (getline(imu_in, temp)) {
        umap.insert(parse_imu_line(temp));
    }

    return umap;
}

std::pair<timestamp, Eigen::Vector3d> DataLoader::parse_imu_line(string line){
    unsigned long long ts;
    double x, y, z;
   
    sscanf(line.c_str(), "%llu, %*u, %lf, %lf, %lf, %*s", &ts, &x, &y, &z);

    timestamp ti {std::chrono::microseconds(ts)};
    Vector3d accel {x, y, z};

    return std::make_pair(ti, accel);
}

