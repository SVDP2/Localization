#ifndef EBIMU_DRIVER__SERIAL_PORT_HPP_
#define EBIMU_DRIVER__SERIAL_PORT_HPP_

#include <boost/asio.hpp>
#include <mutex>
#include <string>
#include <vector>

namespace ebimu_driver
{

class SerialPort
{
public:
  SerialPort();
  ~SerialPort();

  bool open(const std::string & port, uint32_t baudrate, std::string & error);
  void close();
  bool isOpen() const;
  bool writeBytes(const std::vector<uint8_t> & bytes, std::string & error);
  std::vector<uint8_t> readAvailable(size_t max_bytes, int timeout_ms, std::string & error);

private:
  boost::asio::io_context io_;
  boost::asio::serial_port serial_;
  mutable std::mutex mutex_;
};

}  // namespace ebimu_driver

#endif  // EBIMU_DRIVER__SERIAL_PORT_HPP_
