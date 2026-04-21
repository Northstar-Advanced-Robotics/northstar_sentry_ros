#ifndef UART_VISION_LOCALIZATION_MESSAGE_HPP
#define UART_VISION_LOCALIZATION_MESSAGE_HPP

#include <memory>

#include "uart/uart_receive_message.hpp"
#include "uart/uart_send_message.hpp"

namespace src::uart
{
class VisionLocalizationMessage : public UartSendMessage

{
public:
    VisionLocalizationMessage();

    UartMessage::MessageType get_type_id() override;

    VisionLocalizationMessage get_vision_localization_message();

    std::vector<uint8_t> serialize() override;

    float x, y;
};

}  // namespace src::uart

#endif  // UART_AUTO_PATH_MESSAGE_HPP