#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>

template <class Ti, class Tf>
class MapReader {
  public:
    MapReader(const std::string& file, bool verbose = false) {
      try {
        YAML::Node config = YAML::LoadFile(file);

        if(!config[0]["origin"] || !config[1]["dim"] || !config[2]["resolution"] || !config[3]["data"]) {
          printf("Check input format!\n" );
          return;
        }
  
        const std::vector<double>& origin_vec = config[0]["origin"].as<std::vector<double>>();
        for(unsigned int i = 0; i < origin_vec.size(); i++)
          origin_(i) = origin_vec[i];
        if(verbose)
          std::cout << "origin: " << origin_.transpose() << std::endl;

        const std::vector<int>& dim_vec = config[1]["dim"].as<std::vector<int>>();
        for(unsigned int i = 0; i < dim_vec.size(); i++)
          dim_(i) = dim_vec[i];
        if(verbose)
          std::cout << "dim: " << dim_.transpose() << std::endl;

        resolution_ = config[2]["resolution"].as<double>();
        if(verbose)
          std::cout << "resolution: " << resolution_ << std::endl;

        const std::vector<int>& data = config[3]["data"].as<std::vector<int>>();
        data_.resize(data.size());
        for(unsigned int i = 0; i < data.size(); i++)
          data_[i] = data[i] > 0 ? 1 : 0;
        
        exist_ = true;
      } catch (YAML::ParserException& e) {
        //std::cout << e.what() << "\n";
        exist_ = false;
      }
    }

    bool exist() { return exist_; }
    Tf origin() { return origin_; }
    Ti dim() { return dim_; }
    double resolution() { return resolution_; }
    std::vector<signed char> data() { return data_; }
  private:
    Tf origin_;
    Ti dim_;
    double resolution_;
    std::vector<signed char> data_;

    bool exist_ = false;
};
