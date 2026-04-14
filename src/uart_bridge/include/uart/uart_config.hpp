// UART config class:
// stores port, baud, head_byte, and other config vars.

#ifndef UART_CONFIG_HPP
#define UART_CONFIG_HPP
#include <cstdint>
#include <string>

namespace src::uart
{
struct UartConfig
{
    const std::string port = "/dev/ttyTHS1";
    const uint32_t baud = 115000;
};

}  // namespace src::uart

#endif