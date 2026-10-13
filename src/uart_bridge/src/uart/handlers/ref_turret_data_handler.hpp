// Meassage Handlers: extend RxHandler
// constructor needs msg passed in that will be updated by handle method to be refrenced elseware
// implment: get_type_id: return spesific meassage id, get_msg_class: return data type of meassage,
// handle: get data into desirted format

#ifndef REF_TURRET_DATA_HANDLER_HPP
#define REF_TURRET_DATA_HANDLER_HPP
#include <cstdint>

#include "uart/messages/ref_turret_data_message.hpp"
#include "uart/uart_message.hpp"
#include "uart/uart_rx_handler.hpp"

namespace src::uart
{
class RefTurretDataHandler : public UartRxHandler
{
public:
    RefTurretDataHandler(RefTurretDataMessage& msg);

    UartMessage::MessageType get_type_id() override;

    void handle(const std::vector<uint8_t>& bytes, long long rx_timestamp_us) override;
    // should parse the data by calling odomety_message.parse, and any other
    // logic/calculations if needed

    RefTurretDataMessage& ref_turret_data_message;
};

}  // namespace src::uart

#endif