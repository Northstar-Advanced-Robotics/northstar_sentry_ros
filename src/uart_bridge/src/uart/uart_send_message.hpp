// UartSendMessage virtual class: extend meassage
// methods: returns message id, serialize: turns message format to bytes

#ifndef UART_SEND_MESSAGE_HPP
#define UART_SEND_MESSAGE_HPP

#include <cstdint>
#include <vector>

#include "uart_message.hpp"

namespace src::uart
{
class UartSendMessage : public UartMessage
{
public:
    UartSendMessage() = default;
    virtual ~UartSendMessage() = default;

    virtual std::vector<uint8_t> serialize() = 0;
};

}  // namespace src::uart

#endif