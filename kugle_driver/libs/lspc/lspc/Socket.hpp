#ifndef LSPC_BOOST_HPP
#define LSPC_BOOST_HPP

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <stdexcept>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/serial_port.hpp>

#include "lspc/Packet.hpp"
#include "lspc/SocketBase.hpp"

namespace lspc {

class Socket : public SocketBase {
  // Buffer for receiving data on the serial link.
  std::array<uint8_t, 1> read_buffer;

  // Serial port
  boost::asio::io_service ioservice;
  boost::asio::serial_port controller_port =
      boost::asio::serial_port(ioservice);
  std::thread ioservice_thread;
  std::atomic<bool> serial_is_sending;

  // Process incoming data on serial link
  //
  // @brief Reads the serial buffer and dispatches the received payload to the
  // relevant message handling callback function.
  void processSerial(const boost::system::error_code &error,
                     std::size_t bytes_transferred) {
    if (error == boost::system::errc::operation_canceled) {
      return;
    } else if (error) {
      throw std::runtime_error("processSerial: " + error.message());
    }

    uint8_t incoming_byte = read_buffer[0];
    processIncomingByte(incoming_byte);

    // READ THE NEXT PACKET
    // Our job here is done. Queue another read.
    boost::asio::async_read(
        controller_port, boost::asio::buffer(read_buffer),
        std::bind(&Socket::processSerial, this, std::placeholders::_1,
                  std::placeholders::_2));

    return;
  }

  void serialWriteCallback(const boost::system::error_code &error,
                           size_t bytes_transferred) {
    serial_is_sending = false;
  }

 public:
  Socket() : serial_is_sending(false) {}

  Socket(const std::string &com_port_name) : serial_is_sending(false) {
    open(com_port_name);
  }

  ~Socket() {
    ioservice.stop();

    if (ioservice_thread.joinable()) {
      ioservice_thread.join();
    }
  }

  void open(const std::string &com_port_name) {
    if (controller_port.is_open()) {
      return;
    }

    controller_port.open(com_port_name);

    boost::asio::async_read(
        controller_port, boost::asio::buffer(read_buffer),
        std::bind(&Socket::processSerial, this, std::placeholders::_1,
                  std::placeholders::_2));

    // Start the I/O service in its own thread.
    ioservice_thread = std::thread([&] { ioservice.run(); });
  }

  bool isOpen() { return controller_port.is_open(); }

  using SocketBase::send;

  // Send a package with lspc
  //
  // @brief Sends a packaged buffer over the USB serial link.
  //
  // @param type The message type. This is user specific; any type between
  // 1-255.
  // @param payload A vector with the serialized payload to be sent.
  //
  // @return True if the packet was sent.
  bool send(uint8_t type, const std::vector<uint8_t> &payload) override {
    Packet outPacket(type, payload);

    if (!serial_is_sending) {
      serial_is_sending = true;
      boost::asio::async_write(
          controller_port, boost::asio::buffer(outPacket.encodedBuffer()),
          std::bind(&Socket::serialWriteCallback, this, std::placeholders::_1,
                    std::placeholders::_2));
    } else {
      return false;
    }
    return true;
  }
};

}  // namespace lspc

#endif  // LSPC_BOOST_HPP
