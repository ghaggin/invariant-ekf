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
        omega,
        accel,
        accel_bias,
        gps,
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
    void parse_raw(std::string);
    void parse_gps(std::string);

private:
    std::multimap<Timestamp, Data> data_;
};
