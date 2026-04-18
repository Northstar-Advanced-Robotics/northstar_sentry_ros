#include "health_message.hpp"

#include <cstring>

namespace src::uart
{
HealthMessage::HealthMessage() : hp(0.0) {}

UartMessage::MessageType HealthMessage::get_type_id() { return UartMessage::MessageType::HEALTH; }

void HealthMessage::parse(const std::vector<uint8_t>& bytes)
{
    std::memcpy(&hp, bytes.data() + 0, sizeof(uint16_t));
}

HealthMessage HealthMessage::get_health_message() { return *this; }

}  // namespace src::uart