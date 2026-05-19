#include "ebimu_driver/serial_port.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <termios.h>
#include <sys/ioctl.h>

namespace ebimu_driver
{

SerialPort::SerialPort()
: serial_(io_)
{
}

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string & port, uint32_t baudrate, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  error.clear();
  boost::system::error_code ec;

  if (serial_.is_open()) {
    serial_.close(ec);
  }

  serial_.open(port, ec);
  if (ec) {
    error = ec.message();
    return false;
  }

  if (ioctl(serial_.native_handle(), TIOCEXCL) < 0) {
    error = "failed to acquire exclusive serial lock; another process may own " + port;
    serial_.close();
    return false;
  }

  serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate), ec);
  serial_.set_option(boost::asio::serial_port_base::character_size(8), ec);
  serial_.set_option(boost::asio::serial_port_base::parity(
      boost::asio::serial_port_base::parity::none), ec);
  serial_.set_option(boost::asio::serial_port_base::stop_bits(
      boost::asio::serial_port_base::stop_bits::one), ec);
  serial_.set_option(boost::asio::serial_port_base::flow_control(
      boost::asio::serial_port_base::flow_control::none), ec);

  if (ec) {
    error = ec.message();
    serial_.close();
    return false;
  }

  return true;
}

void SerialPort::close()
{
  std::lock_guard<std::mutex> lock(mutex_);
  boost::system::error_code ec;
  if (serial_.is_open()) {
    serial_.cancel(ec);
    serial_.close(ec);
  }
}

bool SerialPort::isOpen() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return serial_.is_open();
}

bool SerialPort::writeBytes(const std::vector<uint8_t> & bytes, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  error.clear();
  if (!serial_.is_open()) {
    error = "serial port is not open";
    return false;
  }

  boost::system::error_code ec;
  boost::asio::write(serial_, boost::asio::buffer(bytes), ec);
  if (ec) {
    error = ec.message();
    return false;
  }
  return true;
}

std::vector<uint8_t> SerialPort::readAvailable(size_t max_bytes, int timeout_ms, std::string & error)
{
  error.clear();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!serial_.is_open()) {
        error = "serial port is not open";
        return {};
      }

      int available = 0;
      if (ioctl(serial_.native_handle(), FIONREAD, &available) < 0) {
        error = "failed to query serial bytes available";
        return {};
      }

      if (available > 0) {
        boost::system::error_code ec;
        std::vector<uint8_t> data(std::min(max_bytes, static_cast<size_t>(available)));
        const size_t n = serial_.read_some(boost::asio::buffer(data), ec);
        if (ec) {
          error = ec.message();
          return {};
        }
        data.resize(n);
        return data;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return {};
}

}  // namespace ebimu_driver
