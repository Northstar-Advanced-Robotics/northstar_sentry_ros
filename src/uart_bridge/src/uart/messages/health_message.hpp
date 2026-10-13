// Message classes like odom: extend Uart send and or recive meassages
// for meassages with both implement
// implement get_type_id with the meassages id, serialize: format into bytes

#ifndef UART_HEALTH_MESSAGE_HPP
#define UART_HEALTH_MESSAGE_HPP

#include <memory>

#include "uart/uart_receive_message.hpp"

namespace src::uart
{
class HealthMessage : public UartReceiveMessage

{
public:
    HealthMessage();

    UartMessage::MessageType get_type_id() override;

    void parse(const std::vector<uint8_t>& msg) override;

    HealthMessage get_health_message();

    uint16_t hp;
};

}  // namespace src::uart

#endif