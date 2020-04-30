#include <DataLoader.hpp>
#include <fstream>
#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <IEKF.hpp>

using namespace Eigen;

size_t count(DataLoader::OutDataType in)
{
    size_t out{0};
    for (auto i{in.first}; i != in.second; ++i, ++out)
        ;
    return out;
}

struct ImuMeas
{
    Vector3d accel;
    Vector3d gyro;
    bool accel_set = false;
    bool gyro_set = false;
};

int main()
{
    auto filepath = "/home/pvbs/Documents/mobrob-project/scripts/AGZ_subset";
    // try to open file, fail otherwise
    std::ifstream test_open(filepath);
    if (!test_open.is_open())
    {
        std::cerr << "File not found or unable to be opened\n";
        std::cerr << "Please provide a correct path to AGZ subset\n";
        exit(1);
    }
    test_open.close();

    // Create data loader and imu measurement
    DataLoader dl(filepath);
    ImuMeas imu;

    // Create iekf
    iekf::IEKF filter;

    while (!dl.complete())
    {
        // Get the next set of data
        auto next_data = dl.next();
        DataLoader::Timestamp ts;

        // Loop through multiple measurements in current data set
        // if needed.
        for (auto i{next_data.first}; i != next_data.second; ++i)
        {
            // Save the timestamp to put in the imu measurement at the bottom
            // of the loop.
            ts = i->first;

            // Test based on data type
            switch (i->second.dt)
            {
                case DataLoader::DataType::omega:
                    // Logic for building imu measurements from data loader
                    if (imu.gyro_set)
                    {
                        throw std::runtime_error("multiple gyro measurements");
                    }
                    else
                    {
                        imu.gyro = i->second.datum;
                        imu.gyro_set = true;
                    }
                    break;

                case DataLoader::DataType::accel:
                    // Logic for building imu measurements from data loader
                    if (imu.accel_set)
                    {
                        throw std::runtime_error("multiple accel measurements");
                    }
                    else
                    {
                        imu.accel = i->second.datum;
                        imu.accel_set = true;
                    }
                    break;

                case DataLoader::DataType::gps:
                    // Add gps measurement to filter
                    filter.addGps(ts, i->second.datum);
                    break;
                case DataLoader::DataType::accel_bias:
                    // do nothing
                    break;
            }
        }

        // Once imu measurement was built add it to the filter
        // Really the correct thing to do would be to rewrite
        // the data loader so it returns pre-built imu measurements...
        if (imu.accel_set && imu.gyro_set)
        {
            // add imu measurement to filter
            filter.addImu(ts, imu.accel, imu.gyro);

            // Reset so that new measurements can be added
            imu.accel_set = false;
            imu.gyro_set = false;
        }

        // get state here if you want it
        filter.getState();
    }
}
