#ifndef FLY_SKY_MESSAGE_HPP
#define FLY_SKY_MESSAGE_HPP

#include <memory>

#include "uart/uart_receive_message.hpp"
#include "uart/uart_send_message.hpp"

namespace src::uart
{
class FLYSKYMessage : public UartReceiveMessage, public UartSendMessage

{
public:
    FLYSKYMessage();

    UartMessage::MessageType get_type_id() override;

    void parse(const std::vector<uint8_t>& msg) override;

    FLYSKYMessage get_alive_message();

    std::vector<uint8_t> serialize() override;

    std::vector<uint8_t> data;
};

}  // namespace src::uart

#endif