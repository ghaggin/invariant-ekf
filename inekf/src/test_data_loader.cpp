#include <iostream>
#include <DataLoader.hpp>
#include <map>

size_t count (DataLoader::OutDataType in){
    size_t out {0};
    for (auto i {in.first}; i != in.second; ++i,++out);
    return out;
}


int main() {
    DataLoader dl("/media/chengjia/h2/data/zurich/AGZ");
    
    while (! dl.complete()){
        auto next_data = dl.next();

        std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(
                        next_data.first->first - dl.first_ts()).count() 
                  << " " << count(next_data);
        
        for (auto i {next_data.first}; i != next_data.second; ++i) {
            switch (i->second.dt) {
                case DataLoader::DataType::omega:
                    std::cout << " omega";
                    break;
                case DataLoader::DataType::accel:
                    std::cout << " accel";
                    break;
                case DataLoader::DataType::accel_bias:
                    std::cout << " accel_bias";
                    break;
                case DataLoader::DataType::gps:
                    std::cout << " gps";
                    break;
            }
        }
        std::cout << std::endl;
    }
}
