// clang-format off
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

    using OutDataType = std::pair<
        std::multimap<Timestamp, Data>::iterator,
        std::multimap<Timestamp, Data>::iterator>;

public:
    DataLoader(std::string data_dir, bool =true);
    OutDataType next();
    bool complete();
    Timestamp first_ts();

private:
    void parse_raw(std::string);
    void parse_gps(std::string);
    void parse_gps_timestamp(std::string);
    void parse_gps_from_gt(std::string);

private:
    std::multimap<Timestamp, Data> data_;
    std::unordered_map<unsigned long long, unsigned long long> gps_time_;

    bool done_ = false;
    Timestamp first_ts_;
    Timestamp next_ts_;
};

inline bool DataLoader::complete() {
    return done_;
}

inline DataLoader::Timestamp DataLoader::first_ts() {
    return first_ts_;
}
