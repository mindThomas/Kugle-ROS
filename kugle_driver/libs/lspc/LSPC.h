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
#include <mutex>

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
    boost::asio::serial_port controller_port;
    std::thread ioservice_thread;
    std::atomic<bool> serial_is_sending;

    // Lock guard mutex
    std::mutex resourceMutex_;

    // Process incoming data on serial link
    //
    // @brief Reads the serial buffer and dispatches the received payload to the
    // relevant message handling callback function.
    void processSerial(const boost::system::error_code &error, std::size_t bytes_transferred);

    void serialWriteCallback(const boost::system::error_code &error, size_t bytes_transferred);

    template <typename SyncReadStream, typename MutableBufferSequence>
    void readWithTimeout(SyncReadStream& s, const MutableBufferSequence& buffers, const boost::asio::deadline_timer::duration_type& expiry_time);

    boost::system::error_code Flush();

   public:
    Socket();
    Socket(const std::string &com_port_name);
    ~Socket();

    void open(const std::string &com_port_name);

    bool send(uint8_t type, const std::vector<uint8_t> &payload);

    bool isOpen();
  };

}  // namespace lspc

#endif  // LSPC_BOOST_HPP
