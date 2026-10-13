#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <asio.hpp>

#include "messages/fly_sky_meassage.hpp"  // Ensure this is the correct path

#include "uart.hpp"

namespace src::uart
{

class IBusReader
{
public:
    // Takes the port name (e.g., "/dev/ttyUSB1") and a pointer to the main trunk UART
    IBusReader(const std::string& port_name, Uart* trunk_uart);
    ~IBusReader();

    void start();
    void stop();

private:
    void receive_loop();

    std::string port_name_;
    Uart* trunk_uart_;  // Pointer to UART 1 (MCB)
    std::atomic<bool> is_running_{false};
    std::thread rx_thread_;
};

}  // namespace src::uart