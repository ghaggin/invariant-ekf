#pragma once

#include <string>
#include <map>
#include <chrono>
#include <eigen3/Eigen/Dense>

class DataLoader {
public:
    using Timestamp = std::chrono::time_point<std::chrono::system_clock,
        std::chrono::duration<double>>;

    enum struct DataType {
        imu,
        gyro,
        gps,
        gt
    };

    struct Data {
        Timestamp ts;
        DataType dt;
        Eigen::Vector3d datum;

        Data(Timestamp ts_, DataType dt_, Eigen::Vector3d datum_) : 
            ts(ts_), dt(dt_), datum(datum_) {}
    };

public:
    DataLoader(std::string data_dir);

private:
    void parse_raw(std::string, std::map<Timestamp, Eigen::Vector3d> &);
    std::pair<Timestamp, Eigen::Vector3d> parse_raw_line(std::string);

    void parse_gps_gt(std::string);
    std::tuple<size_t, Eigen::Vector3d, Eigen::Vector3d>
        parse_gps_gt_line(std::string);

    // -------------------------------------------------------------------------

private:
    std::map<Timestamp, Eigen::Vector3d> imu;
    std::map<Timestamp, Eigen::Vector3d> gyro;
    std::map<Timestamp, Eigen::Vector3d> gps;
    //std::map<size_t, Eigen::Vector3d> gt;
};
