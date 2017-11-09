#ifndef CARTOGRAPHER_IO_DATA_LOGGER_H_
#define CARTOGRAPHER_IO_DATA_LOGGER_H_

#include <map>
#include <vector>

namespace cartographer {
namespace io {

class DataLogger {
 public:
  DataLogger();
  void toCSV(std::string dir);
  void addElement(std::string type, double val);


 private:
  //std::map<std::string, double>& getData();
  std::map<std::string, std::vector<double>> data_;


};

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_DATA_LOGGER_H_
