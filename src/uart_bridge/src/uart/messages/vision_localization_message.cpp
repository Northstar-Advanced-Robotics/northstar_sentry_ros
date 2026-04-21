#include "vision_localization_message.hpp"

#include <cstring>

namespace src::uart
{
VisionLocalizationMessage::VisionLocalizationMessage()
    : x(0),
      y(0)
{
}

UartMessage::MessageType VisionLocalizationMessage::get_type_id()
{
    return UartMessage::MessageType::VISION_LOCALIZION;
}

VisionLocalizationMessage VisionLocalizationMessage::get_vision_localization_message() { return *this; }

std::vector<uint8_t> VisionLocalizationMessage::serialize()
{
    std::vector<uint8_t> bytes(8);

    std::memcpy(bytes.data() + 0, &x, sizeof(x));
    std::memcpy(bytes.data() + 4, &y, sizeof(y));

    return bytes;
}

}  // namespace src::uart