#include "robot_id_handler.hpp"

namespace src::uart
{
RobotIDHandler::RobotIDHandler(RobotIDMessage& msg) : robot_id_message(msg) {}

UartMessage::MessageType RobotIDHandler::get_type_id()
{
    return UartMessage::MessageType::ROBOT_ID;
}

void RobotIDHandler::handle(const std::vector<uint8_t>& bytes, long long rx_timestamp_us)
{
    robot_id_message.parse(bytes);
}
}  // namespace src::uart