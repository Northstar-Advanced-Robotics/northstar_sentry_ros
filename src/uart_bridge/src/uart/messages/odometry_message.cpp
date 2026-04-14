#include "odometry_message.hpp"

#include <cstring>

namespace src::uart
{
OdometryMessage::OdometryMessage()
    // : pos_x(0.0f),
    //   pos_y(0.0f),
    //   pos_z(0.0f),
    :  pitch(0.0f),
      roll(0.0f),
      yaw(0.0f),
      vel_x(0.0f),
      vel_y(0.0f)
    //   vel_z(0.0f)
{
}

UartMessage::MessageType OdometryMessage::get_type_id()
{
    return UartMessage::MessageType::ODOMETRY;
}

void OdometryMessage::parse(const std::vector<uint8_t>& bytes)
{
    // std::memcpy(&pos_x, bytes.data() + 0, sizeof(float));

    // std::memcpy(&pos_y, bytes.data() + 4, sizeof(float));

    // std::memcpy(&pos_z, bytes.data() + 8, sizeof(float));

    std::memcpy(&vel_x, bytes.data() + 0, sizeof(float));

    std::memcpy(&vel_y, bytes.data() + 4, sizeof(float));

    // std::memcpy(&vel_z, bytes.data() + 20, sizeof(float));

    std::memcpy(&pitch, bytes.data() + 8, sizeof(float));

    std::memcpy(&yaw, bytes.data() + 12, sizeof(float));

    std::memcpy(&roll, bytes.data() + 16, sizeof(float));

    // std::memcpy(&pitch_vel, bytes.data() + 36, sizeof(float));

    std::memcpy(&yaw_vel, bytes.data() + 20, sizeof(float));

    // std::memcpy(&roll_vel, bytes.data() + 44, sizeof(float));
}

OdometryMessage OdometryMessage::get_odometry_message() { return *this; }

std::vector<uint8_t> OdometryMessage::serialize()
{
    std::vector<uint8_t> bytes(9 * 4);
    // std::memcpy(bytes.data() + 0, &pos_x, sizeof(pos_x));
    // std::memcpy(bytes.data() + 4, &pos_y, sizeof(pos_y));
    // std::memcpy(bytes.data() + 8, &pos_z, sizeof(pos_z));

    std::memcpy(bytes.data() + 0, &vel_x, sizeof(vel_x));
    std::memcpy(bytes.data() + 4, &vel_y, sizeof(vel_y));
    // std::memcpy(bytes.data() + 20, &vel_z, sizeof(vel_z));

    std::memcpy(bytes.data() + 8, &pitch, sizeof(pitch));
    std::memcpy(bytes.data() + 12, &yaw, sizeof(yaw));
    std::memcpy(bytes.data() + 16, &roll, sizeof(roll));

    // std::memcpy(bytes.data() + 36, &pitch_vel, sizeof(pitch_vel));
    std::memcpy(bytes.data() + 20, &yaw_vel, sizeof(yaw_vel));
    // std::memcpy(bytes.data() + 44, &roll_vel, sizeof(roll_vel));

    return bytes;
}

}  // namespace src::uart