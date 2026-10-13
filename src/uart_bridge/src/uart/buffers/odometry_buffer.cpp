#include "odometry_buffer.hpp"

#include <iostream>

namespace src::uart
{

OdometryBuffer::OdometryBuffer(size_t max_size) : max_capacity(max_size) {}

void OdometryBuffer::clear()
{
    std::lock_guard<std::mutex> lock(mtx);
    buffer.clear();
}

size_t OdometryBuffer::size()
{
    std::lock_guard<std::mutex> lock(mtx);
    return buffer.size();
}

bool OdometryBuffer::is_empty()
{
    std::lock_guard<std::mutex> lock(mtx);
    return buffer.empty();
}

void OdometryBuffer::push(const OdometryMessage& msg)
{
    std::lock_guard<std::mutex> lock(mtx);
    buffer.push_back(msg);

    if (buffer.size() > max_capacity)
    {
        buffer.pop_front();
    }
}

std::optional<OdometryMessage> OdometryBuffer::get_interpolated_state(uint32_t target_time)
{
    std::lock_guard<std::mutex> lock(mtx);

    if (buffer.empty()) return std::nullopt;

    // Edge cases: Target time is outside our known history
    if (target_time <= buffer.front().timestamp)
    {
        // std::cout << "Front" << "\n";
        return buffer.front();
    }
    if (target_time >= buffer.back().timestamp)
    {
        // std::cout << "Back" << "\n";
        return buffer.back();
    }

    // Iterate through the buffer to find the surrounding messages
    for (size_t i = 0; i < buffer.size() - 1; ++i)
    {
        const auto& msg1 = buffer[i];
        const auto& msg2 = buffer[i + 1];

        if (target_time >= msg1.timestamp && target_time <= msg2.timestamp)
        {
            return interpolate(msg1, msg2, target_time);
        }
    }

    return std::nullopt;
}

OdometryMessage OdometryBuffer::interpolate(
    const OdometryMessage& m1,
    const OdometryMessage& m2,
    uint32_t t)
{
    OdometryMessage result;
    result.timestamp = t;

    float dt = static_cast<float>(m2.timestamp - m1.timestamp);
    if (dt == 0.0f) return m1;

    float ratio = static_cast<float>(t - m1.timestamp) / dt;

    // Interpolate Positions
    // result.pos_x = m1.pos_x + ratio * (m2.pos_x - m1.pos_x);
    // result.pos_y = m1.pos_y + ratio * (m2.pos_y - m1.pos_y);
    // result.pos_z = m1.pos_z + ratio * (m2.pos_z - m1.pos_z);

    // Interpolate Angles
    result.yaw = m1.yaw + ratio * (m2.yaw - m1.yaw);
    result.pitch = m1.pitch + ratio * (m2.pitch - m1.pitch);
    result.roll = m1.roll + ratio * (m2.roll - m1.roll);

    // Interpolate Linear Velocities
    result.vel_x = m1.vel_x + ratio * (m2.vel_x - m1.vel_x);
    result.vel_y = m1.vel_y + ratio * (m2.vel_y - m1.vel_y);
    // result.vel_z = m1.vel_z + ratio * (m2.vel_z - m1.vel_z);

    // Interpolate Angular Velocities
    // result.pitch_vel = m1.pitch_vel + ratio * (m2.pitch_vel - m1.pitch_vel);
    // result.yaw_vel = m1.yaw_vel + ratio * (m2.yaw_vel - m1.yaw_vel);
    // result.roll_vel = m1.roll_vel + ratio * (m2.roll_vel - m1.roll_vel);

    return result;
}

}  // namespace src::uart