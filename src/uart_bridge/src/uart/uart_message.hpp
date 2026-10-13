// Meassage class that houses get_type_id

#ifndef UART_MESSAGE_HPP
#define UART_MESSAGE_HPP

namespace src::uart
{
class UartMessage
{
public:
    enum MessageType : uint16_t
    {
        TURRET_DATA = 1,
        ROBOT_ID = 2,
        ALIVE = 3,
        ODOMETRY = 4,
        AUTO_PATH = 5,
        HEALTH = 6,
        REF_TURRET_DATA = 7,
        VISION_LOCALIZION = 8,
        FLY_SKY_DATA = 9
    };

    virtual MessageType get_type_id() = 0;
};

}  // namespace src::uart
#endif