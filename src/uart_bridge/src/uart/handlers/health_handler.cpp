#include "health_handler.hpp"

namespace src::uart
{
HealthHandler::HealthHandler(HealthMessage& msg) : health_message(msg) {}

UartMessage::MessageType HealthHandler::get_type_id() { return UartMessage::MessageType::HEALTH; }
void HealthHandler::handle(const std::vector<uint8_t>& bytes, long long rx_timestamp_us)
{
    health_message.parse(bytes);
}
}  // namespace src::uart