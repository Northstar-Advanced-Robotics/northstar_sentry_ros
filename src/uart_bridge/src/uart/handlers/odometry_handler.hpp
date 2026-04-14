// Meassage Handlers: extend RxHandler
// constructor needs msg passed in that will be updated by handle method to be refrenced elseware
// implment: get_type_id: return spesific meassage id, get_msg_class: return data type of meassage,
// handle: get data into desirted format

#ifndef ODOMETRY_HANDLER_HPP
#define ODOMETRY_HANDLER_HPP
#include <cstdint>

#include "uart/messages/odometry_message.hpp"
#include "uart/uart_message.hpp"
#include "uart/uart_rx_handler.hpp"

namespace src::uart
{
class OdometryHandler : public UartRxHandler
{
public:
    OdometryHandler(OdometryMessage& msg);

    UartMessage::MessageType get_type_id() override;

    void handle(const std::vector<uint8_t>& bytes) override;
    // should parse the data by calling odomety_message.parse, and any other
    // logic/calculations if needed

    OdometryMessage& odometry_message;
};

}  // namespace src::uart

#endif