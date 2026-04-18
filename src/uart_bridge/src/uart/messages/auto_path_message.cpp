#include "auto_path_message.hpp"

#include <cstring>

namespace src::uart
{
AutoPathMessage::AutoPathMessage()
    : start_point_x(0),
      start_point_y(0),
      end_point_x(0),
      end_point_y(0),
      control_point1_x(0),
      control_point1_y(0),
      control_point2_x(0),
      control_point2_y(0)
{
}

UartMessage::MessageType AutoPathMessage::get_type_id()
{
    return UartMessage::MessageType::AUTO_PATH;
}

AutoPathMessage AutoPathMessage::get_auto_path_message() { return *this; }

std::vector<uint8_t> AutoPathMessage::serialize()
{
    std::vector<uint8_t> bytes(9 * 4);

    std::memcpy(bytes.data() + 0, &start_point_x, sizeof(start_point_x));
    std::memcpy(bytes.data() + 4, &start_point_y, sizeof(start_point_y));
    std::memcpy(bytes.data() + 8, &end_point_x, sizeof(end_point_x));
    std::memcpy(bytes.data() + 12, &end_point_y, sizeof(end_point_y));
    std::memcpy(bytes.data() + 16, &control_point1_x, sizeof(control_point1_x));
    std::memcpy(bytes.data() + 20, &control_point1_y, sizeof(control_point1_y));
    std::memcpy(bytes.data() + 24, &control_point2_x, sizeof(control_point2_x));
    std::memcpy(bytes.data() + 28, &control_point2_y, sizeof(control_point2_y));
    std::memcpy(bytes.data() + 32, &length, sizeof(length));

    return bytes;
}

}  // namespace src::uart