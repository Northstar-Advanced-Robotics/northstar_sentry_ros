// Message classes like odom: extend Uart send and or recive meassages
// for meassages with both implement
// implement get_type_id with the meassages id, serialize: format into bytes

#ifndef UART_ODOMETRY_MESSAGE_HPP
#define UART_ODOMETRY_MESSAGE_HPP

#include <memory>

#include "uart/uart_receive_message.hpp"
#include "uart/uart_send_message.hpp"

namespace src::uart
{
class OdometryMessage : public UartReceiveMessage, public UartSendMessage

{
public:
    OdometryMessage();

    UartMessage::MessageType get_type_id() override;

    void parse(const std::vector<uint8_t>& msg) override;

    OdometryMessage get_odometry_message();

    std::vector<uint8_t> serialize() override;

    uint32_t timestamp;
    float vel_x, vel_y, roll, pitch, yaw, pos_x, pos_y, yaw_vel; //pos_z, vel_z, roll_vel, pitch_vel
};

}  // namespace src::uart

#endif