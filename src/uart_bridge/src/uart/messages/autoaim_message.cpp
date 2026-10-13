#include "autoaim_message.hpp"

#include <cstring>

namespace src::uart
{
AutoAimMessage::AutoAimMessage() : yaw_error(0.0f), pitch_error(0.0f), distance(0.0f), target_id(0)
{
}

UartMessage::MessageType AutoAimMessage::get_type_id()
{
    return UartMessage::MessageType::TURRET_DATA;
}

AutoAimMessage AutoAimMessage::get_autoaim_message() { return *this; }

std::vector<uint8_t> AutoAimMessage::serialize()
{
    // 3 floats (12 bytes) + 1 uint16 (2 bytes) = 14 bytes total
    std::vector<uint8_t> bytes(14);

    std::memcpy(bytes.data() + 0, &yaw_error, sizeof(yaw_error));
    std::memcpy(bytes.data() + 4, &pitch_error, sizeof(pitch_error));
    std::memcpy(bytes.data() + 8, &distance, sizeof(distance));
    std::memcpy(bytes.data() + 12, &target_id, sizeof(target_id));

    return bytes;
}

}  // namespace src::uart