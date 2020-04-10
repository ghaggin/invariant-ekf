#pragma once

#include <string>
#include <map>
#include <chrono>
#include <eigen3/Eigen/Dense>

class DataLoader {
public:
    DataLoader(std::string data_dir);

private:
    std::map<std::chrono::time_point<std::chrono::system_clock,
        std::chrono::duration<double>>, Eigen::Vector3d> parse_imu(std::string);
    std::pair<std::chrono::time_point<std::chrono::system_clock,
        std::chrono::duration<double>>, Eigen::Vector3d>
        parse_imu_line(std::string);

    std::map<std::chrono::time_point<std::chrono::system_clock,
        std::chrono::duration<double>>, Eigen::Vector3d> parse_gps(std::string);
    std::pair<std::chrono::time_point<std::chrono::system_clock,
        std::chrono::duration<double>>, Eigen::Vector3d>
        parse_gps_line(std::string);

    // -------------------------------------------------------------------------

    std::map<std::chrono::time_point<std::chrono::system_clock,              
        std::chrono::duration<double>>, Eigen::Vector3d> imu;

    std::map<std::chrono::time_point<std::chrono::system_clock,              
        std::chrono::duration<double>>, Eigen::Vector3d> gyro;
    
    std::map<std::chrono::time_point<std::chrono::system_clock,              
        std::chrono::duration<double>>, Eigen::Vector3d> gps;
};
