#include "uart.hpp"

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
        serial.set_option(asio::serial_port_base::baud_rate(config.baud));
    }
    catch (const std::exception& e)
    {
        std::cerr << "Failed to open serial port: " << e.what() << std::endl;
    }
};

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
    std::vector<uint8_t> header_buffer(4);
    std::vector<uint8_t> payload_buffer; 
    uint8_t byte;

    while(true)
    {
        if (!readBytes(&byte, 1)) {
            continue; 
        }

        if (byte != UART_HEAD_BYTE) {
            continue;
        }

        if (!readBytes(header_buffer.data(), 4)) {
            std::cerr << "Header read timeout/fail. Resetting." << std::endl;
            continue;
        }

        uint16_t dataLength;
        uint8_t seq = header_buffer[2];
        uint8_t crc8_recv = header_buffer[3];
        std::memcpy(&dataLength, &header_buffer[0], sizeof(dataLength));

        // --- CRC8 Check ---
        if (rxCrcEnabled)
        {
            std::vector<uint8_t> crc8_check_data = {UART_HEAD_BYTE};
            crc8_check_data.insert(crc8_check_data.end(), header_buffer.begin(), header_buffer.begin() + 3);

            if (crc::calculate_crc8(crc8_check_data) != crc8_recv)
            {
                std::cerr << "CRC8 mismatch. Dropping packet and resyncing." << std::endl;
                continue;
            }
        }

        size_t payload_size = dataLength + 4; // 2 for type, 2 for crc16
        if (payload_buffer.size() < payload_size) {
            payload_buffer.resize(payload_size);
        }

        if (!readBytes(payload_buffer.data(), payload_size)) {
             std::cerr << "Payload read timeout. Resetting." << std::endl;
             continue;
        }

        // --- CRC16 Check ---
        if (rxCrcEnabled)
        {
            std::vector<uint8_t> crc16_check_data = {UART_HEAD_BYTE};
            crc16_check_data.insert(crc16_check_data.end(), header_buffer.begin(), header_buffer.end());
            
            crc16_check_data.insert(
                crc16_check_data.end(),
                payload_buffer.begin(),
                payload_buffer.begin() + payload_size - 2);

            uint16_t crc16_recv;
            std::memcpy(
                &crc16_recv,
                payload_buffer.data() + payload_size - 2,
                sizeof(crc16_recv));

            if (crc::calculate_crc16(crc16_check_data) != crc16_recv)
            {
                std::cerr << "CRC16 mismatch. Dropping packet." << std::endl;
                continue;
            }
        }

        uint16_t rawMsgType;
        std::memcpy(&rawMsgType, payload_buffer.data(), 2);
        UartMessage::MessageType messageType = static_cast<UartMessage::MessageType>(rawMsgType);

        if (handlers.find(messageType) != handlers.end())
        {
            std::vector<uint8_t> data(payload_buffer.begin() + 2, payload_buffer.begin() + payload_size - 2);
            handlers.at(messageType)->handle(data);
        }
        else
        {
            std::cerr << "No handler for msg type: " << rawMsgType << std::endl;
        }
    }
}

}  // namespace src::uart