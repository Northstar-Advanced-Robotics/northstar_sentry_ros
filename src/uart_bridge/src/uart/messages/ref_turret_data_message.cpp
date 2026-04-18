#include "ref_turret_data_message.hpp"

#include <cstring>

namespace src::uart
{
RefTurretDataMessage::RefTurretDataMessage()
    : bulletSpeed(0.0f),
      bulletsRemaining17(0),
      bulletsRemaining42(0),
      bulletType(BulletType::AMMO_17),
      coolingRate(0),
      firingFreq(0),
      heat17ID1(0),
      heat17ID2(0),
      heat42(0),
      heatLimit(0),
      lastReceivedLaunchingInfoTimestamp(0),
      launchMechanismID(MechanismID::TURRET_17MM_1),
      yaw(0.0f)
{
}

UartMessage::MessageType RefTurretDataMessage::get_type_id()
{
    return UartMessage::MessageType::REF_TURRET_DATA;
}

void RefTurretDataMessage::parse(const std::vector<uint8_t>& bytes)
{
    uint8_t offset = 0;


    std::memcpy(&bulletSpeed, bytes.data() + offset, sizeof(float));
    offset += sizeof(float);

    std::memcpy(&bulletsRemaining17, bytes.data() + offset, sizeof(uint16_t));
    offset += sizeof(uint16_t);

    std::memcpy(&bulletsRemaining42, bytes.data() + offset, sizeof(uint16_t));
    offset += sizeof(uint16_t);

    std::memcpy(&bulletType, bytes.data() + offset, sizeof(BulletType));
    offset += sizeof(BulletType);

    std::memcpy(&coolingRate, bytes.data() + offset, sizeof(uint16_t));
    offset += sizeof(uint16_t);

    std::memcpy(&firingFreq, bytes.data() + offset, sizeof(uint8_t));
    offset += sizeof(uint8_t);

    std::memcpy(&heat17ID1, bytes.data() + offset, sizeof(uint16_t));
    offset += sizeof(uint16_t);

    std::memcpy(&heat17ID2, bytes.data() + offset, sizeof(uint16_t));
    offset += sizeof(uint16_t);

    std::memcpy(&heat42, bytes.data() + offset, sizeof(uint16_t));
    offset += sizeof(uint16_t);

    std::memcpy(&heatLimit, bytes.data() + offset, sizeof(uint16_t));
    offset += sizeof(uint16_t);

    std::memcpy(&lastReceivedLaunchingInfoTimestamp, bytes.data() + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);

    std::memcpy(&launchMechanismID, bytes.data() + offset, sizeof(MechanismID));
    offset += sizeof(MechanismID);

    std::memcpy(&yaw, bytes.data() + offset, sizeof(float));
    offset += sizeof(float);
}

RefTurretDataMessage RefTurretDataMessage::get_ref_turret_data_message() { return *this; }

}  // namespace src::uart