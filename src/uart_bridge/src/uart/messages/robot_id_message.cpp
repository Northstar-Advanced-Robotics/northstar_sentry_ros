#include "robot_id_message.hpp"

#include <cstring>

namespace src::uart
{
RobotIDMessage::RobotIDMessage() {}

UartMessage::MessageType RobotIDMessage::get_type_id()
{
    return UartMessage::MessageType::ROBOT_ID;
}

void RobotIDMessage::parse(const std::vector<uint8_t>& bytes)
{
    uint8_t raw_id = bytes[0];
    robot_id = static_cast<RobotId>(raw_id);
}

RobotIDMessage RobotIDMessage::get_alive_message() { return *this; }

std::vector<uint8_t> RobotIDMessage::serialize()
{
    // No data needed, only msg type.
    std::vector<uint8_t> bytes(0);

    return bytes;
}

}  // namespace src::uart