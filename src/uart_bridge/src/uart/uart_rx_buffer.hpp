#ifndef UART_RX_BUFFER_HPP
#define UART_RX_BUFFER_HPP

#include <cstddef>

namespace src::uart
{
class UartRxBuffer
{
public:
    virtual ~UartRxBuffer() = default;

    virtual void clear() = 0;
    virtual size_t size() = 0;
    virtual bool is_empty() = 0;
};

}  // namespace src::uart

#endif  // UART_RX_BUFFER_HPP