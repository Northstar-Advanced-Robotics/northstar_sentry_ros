// RxHandler virtual class:
// methods: get_type_id, get_msg_class, handle

#ifndef UART_RX_HANDLER_HPP
#define UART_RX_HANDLER_HPP

#include <cstdint>
#include <vector>

#include "uart_message.hpp"

namespace src::uart
{
class UartRxHandler
{
public:
    UartRxHandler() = default;
    virtual ~UartRxHandler() = default;

    virtual UartMessage::MessageType get_type_id() = 0;

    virtual void handle(const std::vector<uint8_t>& bytes) = 0;
};

}  // namespace src::uart
#endif