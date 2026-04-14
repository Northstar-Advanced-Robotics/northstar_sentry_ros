#ifndef UART_ALIVE_MESSAGE_HPP
#define UART_ALIVE_MESSAGE_HPP

#include <memory>

#include "uart/uart_receive_message.hpp"
#include "uart/uart_send_message.hpp"

namespace src::uart
{
class AliveMessage : public UartReceiveMessage, public UartSendMessage

{
public:
    AliveMessage();

    UartMessage::MessageType get_type_id() override;

    void parse(const std::vector<uint8_t>& msg) override;

    AliveMessage get_alive_message();

    std::vector<uint8_t> serialize() override;
};

}  // namespace src::uart

#endif