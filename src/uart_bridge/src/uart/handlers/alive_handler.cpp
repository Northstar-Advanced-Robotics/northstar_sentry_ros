#include "alive_handler.hpp"

namespace src::uart
{
AliveHandler::AliveHandler(AliveMessage& msg) : alive_message(msg) {}

UartMessage::MessageType AliveHandler::get_type_id() { return UartMessage::MessageType::ALIVE; }

void AliveHandler::handle(const std::vector<uint8_t>& bytes)
{
    alive_message.parse(bytes);  // TODO make logic for if online
}
}  // namespace src::uart