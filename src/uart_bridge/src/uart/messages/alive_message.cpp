#include "alive_message.hpp"

#include <cstring>

namespace src::uart
{
AliveMessage::AliveMessage() {}

UartMessage::MessageType AliveMessage::get_type_id() { return UartMessage::MessageType::ALIVE; }

void AliveMessage::parse(const std::vector<uint8_t>& bytes) {}

AliveMessage AliveMessage::get_alive_message() { return *this; }

std::vector<uint8_t> AliveMessage::serialize()
{
    // No data needed, only msg type.
    std::vector<uint8_t> bytes(0);

    return bytes;
}

}  // namespace src::uart