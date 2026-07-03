#ifndef FIELD_SETUP_HPP
#define FIELD_SETUP_HPP

#include <eigen3/Eigen/Dense>

namespace src::AutoPathing
{

class FieldSetup
{
public:
    const float fieldRealSizeX;  // meters
    const float fieldRealSizeY;  // meters
    const float nodeSize;        // meters per grid cell

    const int gridSizeX;  // number of grid cells in X direction
    const int gridSizeY;  // number of grid cells in Y direction

    // in meters, defined as pairs of bottom-left and top-right corners with the origin as the
    // center of the field
    const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> obstacleBounds;

    FieldSetup(
        float fieldRealSizeX,
        float fieldRealSizeY,
        float nodeSizeMeters,
        std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> obstacleBounds)
        : fieldRealSizeX(fieldRealSizeX),
          fieldRealSizeY(fieldRealSizeY),
          nodeSize(nodeSizeMeters),
          gridSizeX((int)(fieldRealSizeX / nodeSize)),
          gridSizeY((int)(fieldRealSizeY / nodeSize)),
          obstacleBounds(obstacleBounds)
    {
    }

    static FieldSetup ARC()
    {
        return FieldSetup(
            12.0f,   // fieldRealSizeX
            8.0f,    // fieldRealSizeY
            0.025f,  // nodeSizeMeters
            {
                // obstacleBounds
                {{-3.75015, -1.82147}, {-3.58015, 1.20393}},  // red big wall
                {{-2.58045, -0.02147}, {-2.41059, 2.00393}},  // red small wall
                {{-3.0, 3.00393}, {3.0, 3.00393}},            // ramp top
                {{-3.78372, 2.00393}, {-1.21628, 2.00393}},   // ramp bottom red
                {{-1.0, -3.3}, {1.0, -2.3}},                  // bottom box
                {{1.21628, 2.00393}, {3.78372, 2.00393}},     // ramp bottom blue
                {{2.41059, -0.02147}, {2.5803, 2.00393}},     // blue small wall
                {{3.58015, -1.82147}, {3.75015, 1.20393}},    // blue big wall
            });
    }

    static FieldSetup ARC_ADJUSTED()
    {
        return FieldSetup(
            12.0f,   // fieldRealSizeX
            8.0f,    // fieldRealSizeY
            0.025f,  // nodeSizeMeters
            {
                // obstacleBounds
                {{-3.75015, -1.82147}, {-3.58015, 1.20393 - 0.05}},  // red big wall
                {{-2.58045, -0.02147 - 0.05}, {-2.41059, 2.00393}},  // red small wall
                {{-3.0, 3.00393}, {3.0, 3.00393}},                   // ramp top
                {{-3.78372, 2.00393 - 0.05}, {-1.21628, 2.00393}},   // ramp bottom red
                {{-1.0, -3.3}, {1.0, -2.3}},                         // bottom box
                {{1.21628, 2.00393 - 0.05}, {3.78372, 2.00393}},     // ramp bottom blue
                {{2.41059, -0.02147 - 0.05}, {2.5803, 2.00393}},     // blue small wall
                {{3.58015, -1.82147}, {3.75015, 1.20393 - 0.05}},    // blue big wall
            });
    }

    static FieldSetup TestField()
    {
        return FieldSetup(
            9.0f,    // fieldRealSizeX
            9.0f,    // fieldRealSizeY
            0.025f,  // nodeSizeMeters
            {
                // obstacleBounds
                //{ { -0.66f, 0.3048f }, { -0.3048f, 0.74 } },  // black/yellow box
            });
    }
};

}  // namespace src::AutoPathing

#endif
