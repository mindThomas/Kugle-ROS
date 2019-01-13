#include "LSPC.h"

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
#include <boost/optional.hpp>

#include "lspc/Packet.hpp"
#include "lspc/SocketBase.hpp"

namespace lspc {

	Socket::Socket() : serial_is_sending(false), controller_port(ioservice) {}

	Socket::Socket(const std::string &com_port_name) : serial_is_sending(false), controller_port(ioservice) {
		open(com_port_name);
	}

	Socket::~Socket() {
		std::lock_guard<std::mutex> lock(resourceMutex_);
		ioservice.stop();

		if (ioservice_thread.joinable()) {
			ioservice_thread.join();
		}
	}

	// Process incoming data on serial link
	//
	// @brief Reads the serial buffer and dispatches the received payload to the
	// relevant message handling callback function.
	void Socket::processSerial(const boost::system::error_code &error,
							   std::size_t bytes_transferred) {
		if (error == boost::system::errc::operation_canceled) {
			return;
		} else if (error) {
			controller_port.close();
			return;
			//throw std::runtime_error("processSerial: " + error.message());
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

	void Socket::serialWriteCallback(const boost::system::error_code &error,
									 size_t bytes_transferred) {
		serial_is_sending = false;
	}

	template <typename SyncReadStream, typename MutableBufferSequence>
	void Socket::readWithTimeout(SyncReadStream& s, const MutableBufferSequence& buffers, const boost::asio::deadline_timer::duration_type& expiry_time)
	{
		boost::optional<boost::system::error_code> timer_result;
		boost::asio::deadline_timer timer(s.get_io_service());
		timer.expires_from_now(expiry_time);
		timer.async_wait([&timer_result] (const boost::system::error_code& error) { timer_result.reset(error); });

		boost::optional<boost::system::error_code> read_result;
		boost::asio::async_read(s, buffers, [&read_result] (const boost::system::error_code& error, size_t) { read_result.reset(error); });

		s.get_io_service().reset();
		while (s.get_io_service().run_one())
		{
			if (read_result)
				timer.cancel();
			else if (timer_result)
				s.cancel();
		}
        s.get_io_service().reset();

		if (*read_result)
			throw boost::system::system_error(*read_result);
	}

	boost::system::error_code Socket::Flush() {
		// From http://mnb.ociweb.com/mnb/MiddlewareNewsBrief-201303.html
#if 0
		boost::system::error_code ec;
#if !defined(BOOST_WINDOWS) && !defined(__CYGWIN__)
		const bool isFlushed =! ::tcflush(controller_port.native(), TCIOFLUSH);
		if (!isFlushed)
			ec = boost::system::error_code(errno,
										   boost::asio::error::get_system_category());
#else
		const bool isFlushed = ::PurgeComm(controller_port.native(),
        PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR);
    if (!isFlushed)
        ec = boost::system::error_code(::GetLastError(),
            boost::asio::error::get_system_category());
#endif
    	return ec;
#else
    	bool continueReading = true;
    	while (continueReading) {
    	    try
    	    {
                readWithTimeout(controller_port, boost::asio::buffer(read_buffer), boost::posix_time::milliseconds(1));
    	    }
			catch (boost::system::system_error& e)
			{
    	        continueReading = false;
    	    }
    	}
#endif
	}

	void Socket::open(const std::string &com_port_name) {
		std::lock_guard<std::mutex> lock(resourceMutex_);
		if (controller_port.is_open()) {
			return;
		}

		controller_port.open(com_port_name);

		if (!controller_port.is_open()) return;

		Flush();

		boost::asio::async_read(
				controller_port, boost::asio::buffer(read_buffer),
				std::bind(&Socket::processSerial, this, std::placeholders::_1,
						  std::placeholders::_2));

		// Start the I/O service in its own thread.
		ioservice_thread = std::thread([&] { ioservice.run(); });
	}

	bool Socket::isOpen() {
		std::lock_guard<std::mutex> lock(resourceMutex_);
		return controller_port.is_open();
	}

	// Send a package with lspc
	//
	// @brief Sends a packaged buffer over the USB serial link.
	//
	// @param type The message type. This is user specific; any type between
	// 1-255.
	// @param payload A vector with the serialized payload to be sent.
	//
	// @return True if the packet was sent.
	bool Socket::send(uint8_t type, const std::vector<uint8_t> &payload) {
		std::lock_guard<std::mutex> lock(resourceMutex_);
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

} // namespace end
