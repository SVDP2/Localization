#include "ebimu_driver/ebimu_parser.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>

namespace ebimu_driver
{
namespace
{
int16_t asSigned16(int32_t raw)
{
  return static_cast<int16_t>(raw & 0xffff);
}

std::vector<std::string> splitComma(const std::string & text)
{
  std::vector<std::string> out;
  std::stringstream ss(text);
  std::string item;
  while (std::getline(ss, item, ',')) {
    out.push_back(item);
  }
  return out;
}
}  // namespace

EbimuParser::EbimuParser(const DriverConfig & config)
: config_(config)
{
}

void EbimuParser::setConfig(const DriverConfig & config)
{
  config_ = config;
  ascii_buffer_.clear();
  binary_buffer_.clear();
}

std::vector<EbimuSample> EbimuParser::parse(const std::vector<uint8_t> & bytes)
{
  if (config_.output_mode == "binary" || config_.output_mode == "hex") {
    return parseBinary(bytes);
  }
  return parseAscii(bytes);
}

ParserStats EbimuParser::stats() const
{
  return stats_;
}

std::vector<EbimuSample> EbimuParser::parseAscii(const std::vector<uint8_t> & bytes)
{
  std::vector<EbimuSample> samples;
  for (const uint8_t byte : bytes) {
    const char c = static_cast<char>(byte);
    if (c == '\n') {
      EbimuSample sample;
      if (decodeAsciiLine(ascii_buffer_, sample)) {
        samples.push_back(sample);
        ++stats_.valid_frames;
      }
      ascii_buffer_.clear();
    } else if (c != '\r') {
      ascii_buffer_.push_back(c);
      if (ascii_buffer_.size() > 512) {
        ascii_buffer_.clear();
        ++stats_.parse_errors;
      }
    }
  }
  return samples;
}

bool EbimuParser::decodeAsciiLine(const std::string & line, EbimuSample & sample)
{
  if (line.empty() || line.front() != '*') {
    return false;
  }

  const auto parts = splitComma(line.substr(1));
  if (parts.size() < expectedValueCount()) {
    ++stats_.parse_errors;
    return false;
  }

  std::vector<double> values;
  values.reserve(parts.size());
  try {
    for (const auto & part : parts) {
      values.push_back(std::stod(part));
    }
  } catch (const std::exception &) {
    ++stats_.parse_errors;
    return false;
  }

  size_t i = 0;
  if (config_.orientation_source == "quaternion") {
    sample.has_quaternion = true;
    const double z = values.at(i++);
    const double y = values.at(i++);
    const double x = values.at(i++);
    const double w = values.at(i++);
    sample.quaternion_xyzw = {x, y, z, w};
  } else {
    sample.has_euler = true;
    sample.euler_deg = {values.at(i++), values.at(i++), values.at(i++)};
  }

  if (config_.enable_gyro) {
    sample.has_gyro = true;
    sample.gyro_dps = {values.at(i++), values.at(i++), values.at(i++)};
  }
  if (config_.enable_accel) {
    sample.has_accel = true;
    sample.accel_g = {values.at(i++), values.at(i++), values.at(i++)};
  }
  if (config_.enable_magnetometer) {
    sample.has_magnetometer = true;
    sample.magnetometer_ut = {values.at(i++), values.at(i++), values.at(i++)};
  }
  if (config_.enable_temperature) {
    sample.has_temperature = true;
    sample.temperature_c = values.at(i++);
  }
  if (config_.enable_timestamp) {
    sample.has_timestamp = true;
    sample.timestamp_ms = static_cast<uint16_t>(std::lround(values.at(i++)));
  }
  return true;
}

std::vector<EbimuSample> EbimuParser::parseBinary(const std::vector<uint8_t> & bytes)
{
  std::vector<EbimuSample> samples;
  binary_buffer_.insert(binary_buffer_.end(), bytes.begin(), bytes.end());

  const size_t value_count = expectedValueCount();
  const size_t frame_len = 2 + value_count * 2 + 2;

  while (binary_buffer_.size() >= frame_len) {
    const std::array<uint8_t, 2> sync_bytes{0x55, 0x55};
    auto sync = std::search(
      binary_buffer_.begin(), binary_buffer_.end(), sync_bytes.begin(), sync_bytes.end());

    if (sync == binary_buffer_.end()) {
      binary_buffer_.clear();
      return samples;
    }
    if (sync != binary_buffer_.begin()) {
      binary_buffer_.erase(binary_buffer_.begin(), sync);
    }
    if (binary_buffer_.size() < frame_len) {
      return samples;
    }

    uint16_t sum = 0;
    for (size_t i = 0; i < frame_len - 2; ++i) {
      sum = static_cast<uint16_t>(sum + binary_buffer_[i]);
    }
    const uint16_t received = static_cast<uint16_t>(
      (static_cast<uint16_t>(binary_buffer_[frame_len - 2]) << 8) | binary_buffer_[frame_len - 1]);

    if (sum != received) {
      ++stats_.checksum_errors;
      binary_buffer_.erase(binary_buffer_.begin());
      continue;
    }

    std::vector<int32_t> raw_values;
    raw_values.reserve(value_count);
    size_t offset = 2;
    for (size_t i = 0; i < value_count; ++i) {
      const int32_t raw = (static_cast<int32_t>(binary_buffer_[offset]) << 8) |
        static_cast<int32_t>(binary_buffer_[offset + 1]);
      raw_values.push_back(raw);
      offset += 2;
    }

    EbimuSample sample;
    if (decodeBinaryValues(raw_values, sample)) {
      samples.push_back(sample);
      ++stats_.valid_frames;
    }
    binary_buffer_.erase(binary_buffer_.begin(), binary_buffer_.begin() + frame_len);
  }

  return samples;
}

bool EbimuParser::decodeBinaryValues(const std::vector<int32_t> & values, EbimuSample & sample)
{
  if (values.size() < expectedValueCount()) {
    ++stats_.parse_errors;
    return false;
  }

  size_t i = 0;
  if (config_.orientation_source == "quaternion") {
    sample.has_quaternion = true;
    const double z = static_cast<double>(asSigned16(values.at(i++))) / 10000.0;
    const double y = static_cast<double>(asSigned16(values.at(i++))) / 10000.0;
    const double x = static_cast<double>(asSigned16(values.at(i++))) / 10000.0;
    const double w = static_cast<double>(asSigned16(values.at(i++))) / 10000.0;
    sample.quaternion_xyzw = {x, y, z, w};
  } else {
    sample.has_euler = true;
    sample.euler_deg = {
      static_cast<double>(asSigned16(values.at(i++))) / 100.0,
      static_cast<double>(asSigned16(values.at(i++))) / 100.0,
      static_cast<double>(asSigned16(values.at(i++))) / 100.0};
  }

  if (config_.enable_gyro) {
    sample.has_gyro = true;
    sample.gyro_dps = {
      static_cast<double>(asSigned16(values.at(i++))) / 10.0,
      static_cast<double>(asSigned16(values.at(i++))) / 10.0,
      static_cast<double>(asSigned16(values.at(i++))) / 10.0};
  }
  if (config_.enable_accel) {
    sample.has_accel = true;
    sample.accel_g = {
      static_cast<double>(asSigned16(values.at(i++))) / 1000.0,
      static_cast<double>(asSigned16(values.at(i++))) / 1000.0,
      static_cast<double>(asSigned16(values.at(i++))) / 1000.0};
  }
  if (config_.enable_magnetometer) {
    sample.has_magnetometer = true;
    sample.magnetometer_ut = {
      static_cast<double>(asSigned16(values.at(i++))) / 10.0,
      static_cast<double>(asSigned16(values.at(i++))) / 10.0,
      static_cast<double>(asSigned16(values.at(i++))) / 10.0};
  }
  if (config_.enable_temperature) {
    sample.has_temperature = true;
    sample.temperature_c = static_cast<double>(asSigned16(values.at(i++))) / 10.0;
  }
  if (config_.enable_timestamp) {
    sample.has_timestamp = true;
    sample.timestamp_ms = static_cast<uint16_t>(values.at(i++) & 0xffff);
  }
  return true;
}

size_t EbimuParser::expectedValueCount() const
{
  size_t count = config_.orientation_source == "quaternion" ? 4 : 3;
  if (config_.enable_gyro) {
    count += 3;
  }
  if (config_.enable_accel) {
    count += 3;
  }
  if (config_.enable_magnetometer) {
    count += 3;
  }
  if (config_.enable_temperature) {
    count += 1;
  }
  if (config_.enable_timestamp) {
    count += 1;
  }
  return count;
}

}  // namespace ebimu_driver
