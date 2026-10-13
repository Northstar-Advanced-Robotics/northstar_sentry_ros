#ifndef ODOMETRY_BUFFER_HPP
#define ODOMETRY_BUFFER_HPP

#include <deque>
#include <mutex>
#include <optional>

#include "uart/messages/odometry_message.hpp"
#include "uart/uart_rx_buffer.hpp"

namespace src::uart
{
class OdometryBuffer : public UartRxBuffer
{
public:
    explicit OdometryBuffer(size_t max_size = 100);
    ~OdometryBuffer() override = default;

    void clear() override;
    size_t size() override;
    bool is_empty() override;

    void push(const OdometryMessage& msg);

    std::optional<OdometryMessage> get_interpolated_state(uint32_t target_time);

private:
    OdometryMessage interpolate(
        const OdometryMessage& m1,
        const OdometryMessage& m2,
        uint32_t target_time);

    std::deque<OdometryMessage> buffer;
    size_t max_capacity;
    std::mutex mtx;
};

}  // namespace src::uart

#endif  // ODOMETRY_BUFFER_HPP