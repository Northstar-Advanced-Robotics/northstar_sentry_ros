#include "odometry_handler.hpp"

namespace src::uart
{
OdometryHandler::OdometryHandler(OdometryMessage& msg) : odometry_message(msg) {}

UartMessage::MessageType OdometryHandler::get_type_id()
{
    return UartMessage::MessageType::ODOMETRY;
}
void OdometryHandler::handle(const std::vector<uint8_t>& bytes) { odometry_message.parse(bytes); }
}  // namespace src::uart