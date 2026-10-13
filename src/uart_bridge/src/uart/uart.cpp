#include "uart.hpp"

#include <iostream>

#include "uart/crc/crc.hpp"

namespace src::uart
{
Uart::Uart(
    const src::uart::UartConfig config,
    std::map<UartMessage::MessageType, std::unique_ptr<src::uart::UartRxHandler>> handlers,
    bool isRxCRCEnforcementEnabled)
    : serial(io),
      rxCrcEnabled(isRxCRCEnforcementEnabled),
      handlers(std::move(handlers)),
      config(config)
{
    try
    {
        serial.open(config.port);

        // 1. Set Baud Rate
        serial.set_option(asio::serial_port_base::baud_rate(config.baud));

        // 2. Force 8 Data Bits (Standard)
        serial.set_option(asio::serial_port_base::character_size(8));

        // 3. Force 1 Stop Bit
        serial.set_option(
            asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));

        // 4. Disable Parity Bit
        serial.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));

        // 5. Disable Hardware/Software Flow Control (CRITICAL FOR BINARY DATA)
        serial.set_option(
            asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
    }
    catch (const std::exception& e)
    {
        std::cerr << "Failed to open serial port: " << e.what() << std::endl;
    }
};

void Uart::close()
{
    is_running_ = false;  // Flag the loop to stop
    std::cout << "Closing\n";

    try
    {
        if (serial.is_open())
        {
            serial.cancel();  // Cancel any pending read/write operations
            serial.close();   // Close the hardware port
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error closing serial port: " << e.what() << std::endl;
    }
}

void Uart::send(std::unique_ptr<src::uart::UartSendMessage> msg)
{
    std::vector<uint8_t> data = msg->serialize();
    std::vector<uint8_t> packet;
    packet.reserve(data.size() + 10);

    // Helper lambda to append any type safely (using raw memory copy)
    auto append_raw = [&](auto value)
    {
        uint8_t* start = reinterpret_cast<uint8_t*>(&value);
        packet.insert(packet.end(), start, start + sizeof(value));
    };

    // 1. Header Byte
    packet.push_back(UART_HEAD_BYTE);

    // 2. Data Length (using the insert trick via lambda)
    uint16_t len = static_cast<uint16_t>(data.size());
    append_raw(len);

    // 3. Seq
    packet.push_back(this->txSeq++);

    // 4. CRC8
    packet.push_back(crc::calculate_crc8(packet));

    // 5. MSG TYPE
    append_raw(static_cast<uint16_t>(msg->get_type_id()));

    // 6. Data
    packet.insert(packet.end(), data.begin(), data.end());

    // 7. CRC16
    uint16_t crc16 = crc::calculate_crc16(packet);
    append_raw(crc16);

    asio::write(serial, asio::buffer(packet));
}

void Uart::receive()
{
    // A persistent buffer to hold data between read cycles
    std::vector<uint8_t> parsing_buffer;

    // A temporary buffer to catch the firehose of bytes from the OS
    std::vector<uint8_t> read_buffer(4096);

    while (is_running_)
    {
        asio::error_code ec;

        // This will block until data arrives OR until serial.close() is called
        size_t len = serial.read_some(asio::buffer(read_buffer), ec);

        // If an error occurred (like the port being closed intentionally)
        if (ec)
        {
            // If we intentionally stopped the thread, or the port closed, break out!
            if (!is_running_ || ec == asio::error::operation_aborted ||
                ec == asio::error::bad_descriptor)
            {
                break;
            }
            continue;  // Minor read errors can continue
        }

        if (len == 0) continue;

        // Add the newly arrived bytes to the back of our parsing window
        parsing_buffer.insert(parsing_buffer.end(), read_buffer.begin(), read_buffer.begin() + len);

        // Process as many full packets as we can find in the buffer
        while (parsing_buffer.size() >= 5)  // 5 is the minimum size for a header
        {
            // 1. Align to the header byte (0xA5)
            auto sync_it = std::find(parsing_buffer.begin(), parsing_buffer.end(), UART_HEAD_BYTE);

            if (sync_it != parsing_buffer.begin())
            {
                // Erase any garbage bytes that came before the header
                parsing_buffer.erase(parsing_buffer.begin(), sync_it);
            }

            // After erasing garbage, do we still have enough for a header?
            if (parsing_buffer.size() < 5) break;

            // 2. Read the data length
            // Length is a uint16_t located at offset 1, right after the 0xA5 byte
            uint16_t dataLength;
            std::memcpy(&dataLength, &parsing_buffer[1], sizeof(dataLength));

            // Calculate total expected packet size
            // Header(5) + Type(2) + Payload(dataLength) + CRC16(2) = dataLength + 9
            size_t total_packet_size = dataLength + 9;

            // 3. Do we have the entire packet yet?
            if (parsing_buffer.size() < total_packet_size)
            {
                // Break out of the parsing loop and wait for more bytes from read_some
                break;
            }

            // 4. We have a full packet! Copy it out so we can process it safely.
            std::vector<uint8_t> packet(
                parsing_buffer.begin(),
                parsing_buffer.begin() + total_packet_size);

            // Immediately erase it from the parsing buffer so the next loop can find the next
            // packet
            parsing_buffer.erase(
                parsing_buffer.begin(),
                parsing_buffer.begin() + total_packet_size);

            // --- PACKET VALIDATION & DISPATCH ---

            // CRC8 Check (Header bytes 0 to 3)
            if (rxCrcEnabled)
            {
                std::vector<uint8_t> crc8_check_data(packet.begin(), packet.begin() + 4);
                uint8_t crc8_recv = packet[4];

                if (crc::calculate_crc8(crc8_check_data) != crc8_recv)
                {
                    std::cerr << "CRC8 mismatch. Dropping packet." << std::endl;
                    continue;  // Moves to the next packet in the parsing loop
                }
            }

            // CRC16 Check (Everything except the last 2 bytes)
            if (rxCrcEnabled)
            {
                std::vector<uint8_t> crc16_check_data(packet.begin(), packet.end() - 2);
                uint16_t crc16_recv;
                std::memcpy(&crc16_recv, &packet[total_packet_size - 2], sizeof(crc16_recv));

                if (crc::calculate_crc16(crc16_check_data) != crc16_recv)
                {
                    std::cerr << "CRC16 mismatch. Dropping packet." << std::endl;
                    continue;
                }
            }

            // 5. Dispatch to Handler
            uint16_t rawMsgType;
            // Message type is located right after the 5-byte header
            std::memcpy(&rawMsgType, &packet[5], sizeof(rawMsgType));
            UartMessage::MessageType messageType =
                static_cast<UartMessage::MessageType>(rawMsgType);

            if (handlers.find(messageType) != handlers.end())
            {
                // Grab the Jetson arrival time as close to extraction as possible
                long long rx_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                           std::chrono::system_clock::now().time_since_epoch())
                                           .count();

                // Extract just the payload data (skip Header(5) + Type(2), end before CRC16(2))
                std::vector<uint8_t> payload_data(packet.begin() + 7, packet.end() - 2);

                handlers.at(messageType)->handle(payload_data, rx_time_us);
            }
            else
            {
                // std::cerr << "No handler for msg type: " << rawMsgType << std::endl;
            }
        }
    }
}
}  // namespace src::uart
