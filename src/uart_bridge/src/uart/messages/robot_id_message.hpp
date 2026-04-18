#ifndef ROBOT_ID_MESSAGE_HPP
#define ROBOT_ID_MESSAGE_HPP

#include <memory>

#include "uart/uart_receive_message.hpp"
#include "uart/uart_send_message.hpp"

namespace src::uart
{
class RobotIDMessage : public UartReceiveMessage, public UartSendMessage

{
public:
    RobotIDMessage();

    UartMessage::MessageType get_type_id() override;

    void parse(const std::vector<uint8_t>& msg) override;

    RobotIDMessage get_alive_message();

    std::vector<uint8_t> serialize() override;

    enum class RobotId : uint8_t
    {
        INVALID = 0,

        RED_HERO = 1,
        RED_ENGINEER = 2,
        RED_SOLDIER_1 = 3,
        RED_SOLDIER_2 = 4,
        RED_SOLDIER_3 = 5,
        RED_DRONE = 6,
        RED_SENTINEL = 7,
        RED_DART = 8,
        RED_RADAR_STATION = 9,

        BLUE_HERO = 101,
        BLUE_ENGINEER = 102,
        BLUE_SOLDIER_1 = 103,
        BLUE_SOLDIER_2 = 104,
        BLUE_SOLDIER_3 = 105,
        BLUE_DRONE = 106,
        BLUE_SENTINEL = 107,
        BLUE_DART = 108,
        BLUE_RADAR_STATION = 109
    };

    RobotId robot_id = RobotId::INVALID;
};

}  // namespace src::uart

#endif