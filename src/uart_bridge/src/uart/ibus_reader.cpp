#include "ibus_reader.hpp"

#include <iostream>

namespace src::uart
{

IBusReader::IBusReader(const std::string& port_name, Uart* trunk_uart)
    : port_name_(port_name),
      trunk_uart_(trunk_uart)
{
}

IBusReader::~IBusReader() { stop(); }

void IBusReader::start()
{
    if (is_running_) return;
    is_running_ = true;
    std::cout << "START" << std::endl;
    rx_thread_ = std::thread(&IBusReader::receive_loop, this);
}

void IBusReader::stop()
{
    is_running_ = false;
    std::cout << "STOP" << std::endl;
    if (rx_thread_.joinable())
    {
        rx_thread_.join();
    }
}

void IBusReader::receive_loop()
{
    asio::io_context io;
    asio::serial_port ibus_serial(io);

    try
    {
        ibus_serial.open(port_name_);
        ibus_serial.set_option(asio::serial_port_base::baud_rate(115200));
        ibus_serial.set_option(asio::serial_port_base::character_size(8));
        ibus_serial.set_option(
            asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
        ibus_serial.set_option(
            asio::serial_port_base::parity(asio::serial_port_base::parity::none));
    }
    catch (const std::exception& e)
    {
        std::cerr << "Failed to open i-BUS port " << port_name_ << ": " << e.what() << std::endl;
        return;
    }

    std::vector<uint8_t> frame_buffer(32);

    while (is_running_)
    {
        uint8_t byte;
        asio::error_code ec;

        // 1. Read exactly one byte to hunt for the 0x20 header
        size_t len = asio::read(ibus_serial, asio::buffer(&byte, 1), ec);

        if (ec)
        {
            continue;  // Handle disconnects or minor read errors
        }
        // 2. If we find the length header (0x20 = 32 bytes)
        if (byte == 0x20)
        {
            frame_buffer[0] = byte;

            // 3. Read the remaining 31 bytes of the frame
            asio::read(ibus_serial, asio::buffer(frame_buffer.data() + 1, 31), ec);

            if (ec) continue;

            // 4. Validate the second header byte (Command ID)
            if (frame_buffer[1] == 0x40)
            {
                // 5. Validate Checksum (0xFFFF minus the sum of bytes 0 to 29)
                uint16_t calculated_checksum = 0xFFFF;
                for (int i = 0; i < 30; i++)
                {
                    calculated_checksum -= frame_buffer[i];
                }

                // Extract received checksum (Little-Endian)
                uint16_t received_checksum = frame_buffer[30] | (frame_buffer[31] << 8);

                if (calculated_checksum == received_checksum)
                {
                    // 6. VALID FRAME! Wrap it in your custom message and send to MCB
                    if (trunk_uart_)
                    {
                        FLYSKYMessage msg;
                        msg.parse(frame_buffer);  // Store the raw 32 bytes

                        // Send it securely using the thread-safe send function on UART 1
                        trunk_uart_->send(std::make_unique<FLYSKYMessage>(msg));
                    }
                }
                else
                {
                    std::cerr << "i-BUS Checksum Failed!" << std::endl;
                }
            }
        }
    }

    ibus_serial.close();
}

}  // namespace src::uart