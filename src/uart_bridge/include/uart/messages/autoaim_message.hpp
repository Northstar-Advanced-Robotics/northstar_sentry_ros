// Message classes like odom: extend Uart send and or recive meassages
// for meassages with both implement
// implement get_type_id with the meassages id, serialize: format into bytes

#ifndef UART_AUTO_AIM_MESSAGE_HPP
#define UART_AUTO_AIM_MESSAGE_HPP

#include <memory>

#include "uart/uart_receive_message.hpp"
#include "uart/uart_send_message.hpp"

namespace src::uart
{
class AutoAimMessage : public UartSendMessage

{
public:
    AutoAimMessage();

    UartMessage::MessageType get_type_id() override;

    AutoAimMessage get_autoaim_message();

    std::vector<uint8_t> serialize() override;

    float yaw_error, pitch_error, distance;
    uint16_t target_id;
};

}  // namespace src::uart

#endif