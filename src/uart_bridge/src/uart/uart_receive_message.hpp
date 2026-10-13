// UartReciveMessage virtual class: extend meassage
// methods: returns message id, parse: turns bytes into formatted data

#ifndef UART_RECEIVE_MESSAGE_HPP
#define UART_RECEIVE_MESSAGE_HPP

#include <cstdint>
#include <memory>
#include <vector>

#include "uart_message.hpp"

namespace src::uart
{
class UartReceiveMessage : public UartMessage
{
public:
    UartReceiveMessage() = default;

    virtual ~UartReceiveMessage() = default;

    virtual void parse(const std::vector<uint8_t>& msg) = 0;
};

}  // namespace src::uart

#endif
