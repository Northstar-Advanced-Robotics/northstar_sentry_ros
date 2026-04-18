// uart class:
// constructor takes in UARTConfig, msg_handlers
// methods: send: call sereilize on SendMessage and construct a dji message anD send it,
// receive: loop until head byte, grab header and look at data length, get data id and data. With
// data id get correct handler for dict, then use get_msg_class and call .parse passing in the raw
// bytes. Now call handler.handle() passing in the parced data.

#ifndef UART_HPP
#define UART_HPP
#include <iostream>
#include <map>
#include <memory>

#include <asio.hpp>
#include <asio/serial_port.hpp>

#include "uart_config.hpp"
#include "uart_rx_handler.hpp"
#include "uart_send_message.hpp"

namespace src::uart
{
class Uart
{
    static constexpr uint8_t UART_HEAD_BYTE = 0xA5;

    static constexpr int HEADER_SIZE = 5;
    static constexpr int CMD_ID_SIZE = 2;
    static constexpr int CRC16_SIZE = 2;
    static constexpr int LEN_OFFSET = 1;
    static constexpr int CRC8_OFFSET = 4;

public:
    Uart(
        const src::uart::UartConfig config,
        std::map<UartMessage::MessageType, std::unique_ptr<src::uart::UartRxHandler>> handlers,
        bool isRxCRCEnforcementEnabled);

    void close();

    void send(std::unique_ptr<src::uart::UartSendMessage> msg);

    void receive();

    std::size_t readBytes(uint8_t *data, std::size_t length)
    {
        asio::error_code ec;

        // This blocks until 'size' bytes are received OR an error occurs.
        size_t len = asio::read(serial, asio::buffer(data, length), ec);

        if (ec)
        {
            // Returns the number of bytes actually read before the error occurred
            return len;
        }

        return len;  // Should equal 'size' on success
    }

private:
    std::atomic<bool> is_running_{true};

    bool rxCrcEnabled;

    uint8_t txSeq = 0;

    asio::io_context io;
    asio::serial_port serial;

    std::vector<uint8_t> rxBuffer;

    std::map<UartMessage::MessageType, std::unique_ptr<src::uart::UartRxHandler>> handlers;
    src::uart::UartConfig config;
};

}  // namespace src::uart

#endif