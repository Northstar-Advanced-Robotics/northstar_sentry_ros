#include "fly_sky_meassage.hpp"

#include <cstring>

namespace src::uart
{
FLYSKYMessage::FLYSKYMessage() {}

UartMessage::MessageType FLYSKYMessage::get_type_id()
{
    return UartMessage::MessageType::FLY_SKY_DATA;
}

void FLYSKYMessage::parse(const std::vector<uint8_t>& bytes) { this->data = bytes; }

FLYSKYMessage FLYSKYMessage::get_alive_message() { return *this; }

std::vector<uint8_t> FLYSKYMessage::serialize() { return data; }

}  // namespace src::uart