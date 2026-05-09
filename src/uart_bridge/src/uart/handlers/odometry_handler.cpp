#include "odometry_handler.hpp"
#include <iostream>

#include <atomic>

extern std::atomic<long long> jetson_to_mcb_offset_us;

namespace src::uart
{

OdometryHandler::OdometryHandler(OdometryMessage& msg) : odometry_message(msg) {}

UartMessage::MessageType OdometryHandler::get_type_id()
{
    return UartMessage::MessageType::ODOMETRY;
}
void OdometryHandler::handle(const std::vector<uint8_t>& bytes, long long rx_timestamp_us)
{
    odometry_message.parse(bytes);

    long long current_raw_offset = rx_timestamp_us - odometry_message.timestamp;

    long long previous_offset = jetson_to_mcb_offset_us.load();

    if (previous_offset == 0)
    {
        jetson_to_mcb_offset_us.store(current_raw_offset);
    }
    else
    {
        if (current_raw_offset < previous_offset){
            jetson_to_mcb_offset_us.store(current_raw_offset);
        } else{
            double alpha = 0.05;

            long long new_offset = previous_offset +
                static_cast<long long>(alpha * (current_raw_offset - previous_offset));

            jetson_to_mcb_offset_us.store(new_offset);
        }

    }
}
}  // namespace src::uart