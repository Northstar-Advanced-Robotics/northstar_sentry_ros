// (X,Y) for start and end points and for each control point
#ifndef UART_AUTO_PATH_MESSAGE_HPP
#define UART_AUTO_PATH_MESSAGE_HPP

#include <memory>

#include "uart/uart_receive_message.hpp"
#include "uart/uart_send_message.hpp"

namespace src::uart
{
class AutoPathMessage : public UartSendMessage

{
public:
    AutoPathMessage();

    UartMessage::MessageType get_type_id() override;

    AutoPathMessage get_auto_path_message();

    std::vector<uint8_t> serialize() override;

    float start_point_x, start_point_y;
    float end_point_x, end_point_y;
    float control_point1_x, control_point1_y;
    float control_point2_x, control_point2_y;
    float length;
};

}  // namespace src::uart

#endif  // UART_AUTO_PATH_MESSAGE_HPP