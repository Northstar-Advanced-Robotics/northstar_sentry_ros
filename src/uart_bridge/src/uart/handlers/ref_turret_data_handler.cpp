#include "ref_turret_data_handler.hpp"

namespace src::uart
{
RefTurretDataHandler::RefTurretDataHandler(RefTurretDataMessage& msg) : ref_turret_data_message(msg)
{
}

UartMessage::MessageType RefTurretDataHandler::get_type_id()
{
    return UartMessage::MessageType::REF_TURRET_DATA;
}
void RefTurretDataHandler::handle(const std::vector<uint8_t>& bytes, long long rx_timestamp_us)
{
    ref_turret_data_message.parse(bytes);
}
}  // namespace src::uart