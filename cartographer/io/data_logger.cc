#include "data_logger.h"

#include <cstdio>
#include <ctime>
#include <iostream>
#include <fstream>
#include <ostream>
#include <sstream>
#include <iomanip>

cartographer::io::DataLogger::DataLogger(){

}


void cartographer::io::DataLogger::toCSV(std::string dir){
 /* std::ofstream myfile;
  std::string log_file_path;
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream oss;
  oss << std::put_time(&tm, "_%m-%d-%Y_%H-%M-%S");
  std::string str = oss.str();
  log_file_path = dir  + ".csv";
  myfile.open (log_file_path);
  for(auto const &data_serie: data_) {
    myfile <<data_serie.first<<",";
    for(auto const &serie_element : data_serie.second) {
       myfile << std::setprecision (15)<<serie_element<<",";
    }
    myfile << "\n";
  }

  myfile.close();*/
  //LOG(INFO)<<"wrote file: "<<log_file_path;

}




void cartographer::io::DataLogger::addElement(std::string type, double val){
  /*auto search = data_.find(type);
  if(search != data_.end()) {
      search->second.push_back(val);
  }
  else {
      data_.insert(std::pair<std::string, std::vector<double>>(type, {val}));
  }
*/
}
