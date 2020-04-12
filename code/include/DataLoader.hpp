#pragma once

#include <string>
#include <map>
#include <chrono>
#include <eigen3/Eigen/Dense>

class DataLoader {
public:
    using Timestamp = std::chrono::time_point<std::chrono::system_clock,
        std::chrono::duration<double>>;
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
    std::map<size_t, Eigen::Vector3d> gps;
    std::map<size_t, Eigen::Vector3d> gt;
};
