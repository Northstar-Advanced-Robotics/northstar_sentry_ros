#ifndef ALIVE_HANDLER_HPP
#define ALIVE_HANDLER_HPP
#include <cstdint>

#include "uart/messages/alive_message.hpp"
#include "uart/uart_message.hpp"
#include "uart/uart_rx_handler.hpp"

namespace src::uart
{
class AliveHandler : public UartRxHandler
{
public:
    AliveHandler(AliveMessage& msg);

    UartMessage::MessageType get_type_id() override;

    void handle(const std::vector<uint8_t>& bytes) override;

    AliveMessage& alive_message;
};

}  // namespace src::uart

#endif