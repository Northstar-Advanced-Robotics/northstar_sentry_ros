#ifndef ALIVE_HANDLER_HPP
#define ALIVE_HANDLER_HPP
#include <cstdint>

#include "uart/messages/robot_id_message.hpp"
#include "uart/uart_message.hpp"
#include "uart/uart_rx_handler.hpp"

namespace src::uart
{
class RobotIDHandler : public UartRxHandler
{
public:
    RobotIDHandler(RobotIDMessage& msg);

    UartMessage::MessageType get_type_id() override;

    void handle(const std::vector<uint8_t>& bytes, long long rx_timestamp_us) override;

    RobotIDMessage& robot_id_message;
};

}  // namespace src::uart

#endif