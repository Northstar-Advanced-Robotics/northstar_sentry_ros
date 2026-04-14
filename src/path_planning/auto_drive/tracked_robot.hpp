#ifndef TRACKED_ROBOT_HPP
#define TRACKED_ROBOT_HPP

#include <eigen3/Eigen/Dense>

namespace src::AutoPathing
{

class TrackedRobot
{
public:
    Eigen::Vector2i position;
    const int nodeRadius;

    TrackedRobot(Eigen::Vector2i pos, int radius)
        : position(pos), nodeRadius(radius) {}

    inline bool isExpired(int64_t currentTimeMS) const {
        return (currentTimeMS - expireTimeMS) > EXPIRE_TIMER_MS;
    }

    inline void refreshExpireTimer(int64_t currentTimeMS) {
        expireTimeMS = currentTimeMS + EXPIRE_TIMER_MS;
    }

private:
    static constexpr int64_t EXPIRE_TIMER_MS = 5000;
    int64_t expireTimeMS;

};

} // namespace src::astar

#endif
