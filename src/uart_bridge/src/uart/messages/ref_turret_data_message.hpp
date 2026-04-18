// Message classes like odom: extend Uart send and or recive meassages
// for meassages with both implement
// implement get_type_id with the meassages id, serialize: format into bytes

#ifndef UART_REF_TURRET_DATA_MESSAGE_HPP
#define UART_REF_TURRET_DATA_MESSAGE_HPP

#include <memory>

#include "uart/uart_receive_message.hpp"

namespace src::uart
{
class RefTurretDataMessage : public UartReceiveMessage

{
public:
    RefTurretDataMessage();

    UartMessage::MessageType get_type_id() override;

    void parse(const std::vector<uint8_t>& msg) override;

    RefTurretDataMessage get_ref_turret_data_message();

    enum MechanismID
        {
            TURRET_17MM_1 = 1,  ///< 17mm barrel ID 1
            TURRET_17MM_2 = 2,  ///< 17mm barrel ID 2
            TURRET_42MM = 3,    ///< 42mm barrel
        };

    enum BulletType
        {
            AMMO_17 = 1,  ///< 17 mm projectile ammo.
            AMMO_42 = 2,  ///< 42 mm projectile ammo.
        };


    float bulletSpeed;

    uint16_t bulletsRemaining17;

    uint16_t bulletsRemaining42;

    BulletType bulletType;

    uint16_t coolingRate;

    uint8_t firingFreq;

    uint16_t heat17ID1;

    uint16_t heat17ID2;

    uint16_t heat42;

    uint16_t heatLimit;

    uint32_t lastReceivedLaunchingInfoTimestamp;

    MechanismID launchMechanismID;

    float yaw;
};


}  // namespace src::uart

#endif