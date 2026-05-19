#ifndef EBIMU_DRIVER__EBIMU_PARSER_HPP_
#define EBIMU_DRIVER__EBIMU_PARSER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "ebimu_driver/ebimu_sample.hpp"

namespace ebimu_driver
{

class EbimuParser
{
public:
  explicit EbimuParser(const DriverConfig & config);

  void setConfig(const DriverConfig & config);
  std::vector<EbimuSample> parse(const std::vector<uint8_t> & bytes);
  ParserStats stats() const;

private:
  std::vector<EbimuSample> parseAscii(const std::vector<uint8_t> & bytes);
  std::vector<EbimuSample> parseBinary(const std::vector<uint8_t> & bytes);
  bool decodeAsciiLine(const std::string & line, EbimuSample & sample);
  bool decodeBinaryValues(const std::vector<int32_t> & values, EbimuSample & sample);
  size_t expectedValueCount() const;

  DriverConfig config_;
  ParserStats stats_;
  std::string ascii_buffer_;
  std::vector<uint8_t> binary_buffer_;
};

}  // namespace ebimu_driver

#endif  // EBIMU_DRIVER__EBIMU_PARSER_HPP_
