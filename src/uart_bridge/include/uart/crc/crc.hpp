// crc.h
#pragma once

#include <cstdint>
#include <vector>

namespace src::uart::crc
{
uint8_t calculate_crc8(const std::vector<uint8_t>& data);
uint16_t calculate_crc16(const std::vector<uint8_t>& data);
}  // namespace src::uart::crc