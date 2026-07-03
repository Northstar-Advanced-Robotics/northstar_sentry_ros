#ifndef THETA_STAR_HPP
#define THETA_STAR_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>  // For abs
#include <queue>
#include <unordered_map>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "field_setup.hpp"
#include "tracked_robot.hpp"

namespace src::AutoPathing
{

struct Vector2iHash
{
    std::size_t operator()(const Eigen::Vector2i& v) const
    {
        std::size_t seed = 0;
        seed ^= std::hash<int>{}(v.x()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>{}(v.y()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

using NodePair = std::pair<float, Eigen::Vector2i>;

struct CompareNode
{
    bool operator()(const NodePair& a, const NodePair& b) const { return a.first > b.first; }
};

class ThetaStar
{
private:
    static const FieldSetup fieldSetup;

    static constexpr float sentryRadius = 0.34;  // meters
    static constexpr int nodeTolerance = 1;      // nodes
    const int robotNodeRadius;

    std::vector<std::byte> obstacleGrid;
    std::byte* obstacleGridPtr;

    std::vector<std::byte> workingGrid;
    std::byte* workingGridPtr;

    std::vector<TrackedRobot> dynamicObstacles;

    inline void copyObstaclesToWorkingGrid()
    {
        std::copy(obstacleGrid.begin(), obstacleGrid.end(), workingGrid.begin());
    }

    inline std::byte& getObstacleGrid(int x, int y)
    {
        return obstacleGridPtr[x + y * fieldSetup.gridSizeX];
    }

    inline void setObstacleGrid(int x, int y, std::byte newValue)
    {
        obstacleGridPtr[x + y * fieldSetup.gridSizeX] = newValue;
    }

public:
    ThetaStar()
        : obstacleGrid(fieldSetup.gridSizeX * fieldSetup.gridSizeY, std::byte{0}),
          workingGrid(fieldSetup.gridSizeX * fieldSetup.gridSizeY, std::byte{0}),
          robotNodeRadius((int)(sentryRadius / fieldSetup.nodeSize + 0.5f))
    {
        obstacleGridPtr = obstacleGrid.data();
        workingGridPtr = workingGrid.data();

        createObstacles();
        growObstacles(robotNodeRadius + nodeTolerance);
        growEdges(robotNodeRadius + nodeTolerance);

        copyObstaclesToWorkingGrid();
    }

    int GetSizeX() const { return fieldSetup.gridSizeX; }
    int GetSizeY() const { return fieldSetup.gridSizeY; }

    float GetRealSizeX() const { return fieldSetup.fieldRealSizeX; }
    float GetRealSizeY() const { return fieldSetup.fieldRealSizeY; }

    inline std::byte& getWorkingGrid(int x, int y)
    {
        return workingGridPtr[x + y * fieldSetup.gridSizeX];
    }

    inline void setWorkingGrid(int x, int y, std::byte newValue)
    {
        workingGridPtr[x + y * fieldSetup.gridSizeX] = newValue;
    }

    bool isNodeValidAndEmpty(Eigen::Vector2i point)
    {
        if (point.x() < 0 || point.x() >= fieldSetup.gridSizeX || point.y() < 0 ||
            point.y() >= fieldSetup.gridSizeY)
        {
            return false;
        }
        return (getWorkingGrid(point.x(), point.y()) == std::byte{0});
    }

    void updateDynamicObstacles()
    {
        if (dynamicObstacles.empty())
        {
            return;
        }

        copyObstaclesToWorkingGrid();

        int64_t currentTimeMS =
            std::chrono::steady_clock::now().time_since_epoch().count() / 1'000'000;

        for (int i = 0; i < dynamicObstacles.size(); i++)
        {
            if (!dynamicObstacles[i].isExpired(currentTimeMS))
            {
                TrackedRobot& robot = dynamicObstacles[i];
                DrawCircle(robot.position, robot.nodeRadius + robotNodeRadius, workingGridPtr);
            }
        }
    }

    std::vector<Eigen::Vector2i> FindPath(Eigen::Vector2i start, Eigen::Vector2i end)
    {
        std::priority_queue<NodePair, std::vector<NodePair>, CompareNode> frontier;

        std::unordered_map<Eigen::Vector2i, Eigen::Vector2i, Vector2iHash> cameFrom;
        std::unordered_map<Eigen::Vector2i, float, Vector2iHash> costSoFar;  // g_score

        frontier.push({0.0f, start});

        cameFrom[start] = start;
        costSoFar[start] = 0;

        while (!frontier.empty())
        {
            NodePair top = frontier.top();
            frontier.pop();
            Eigen::Vector2i current = top.second;

            if (current == end)
            {
                break;
            }

            Eigen::Vector2i dirs[] =
                {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1}};

            for (const auto& dir : dirs)
            {
                Eigen::Vector2i neighbor = current + dir;

                // Skip if out of bounds or inside a Hard Wall (1)
                if (!isNodeValidAndEmpty(neighbor))
                {
                    continue;
                }

                // --- THETA* LOGIC ---
                // 1. Check if we have a parent (to be the "grandparent")
                // 2. Check if the "grandparent" has Line-Of-Sight to the neighbor
                bool loS = false;
                if (cameFrom.find(current) != cameFrom.end())
                {
                    Eigen::Vector2i parent = cameFrom[current];
                    // Don't check LOS if parent is current (start node)
                    if (parent != current)
                    {
                        if (lineOfSight(parent, neighbor))
                        {
                            // Path 1: Grandparent -> Neighbor (Straight Line Shortcut)
                            float newCost =
                                costSoFar[parent] + (parent - neighbor).cast<float>().norm();
                            if (costSoFar.find(neighbor) == costSoFar.end() ||
                                newCost < costSoFar[neighbor])
                            {
                                costSoFar[neighbor] = newCost;
                                float priority = newCost + heuristic(neighbor, end);
                                frontier.push({priority, neighbor});
                                cameFrom[neighbor] = parent;  // Point to Grandparent
                            }
                            loS = true;
                        }
                    }
                }

                // Path 2: Current -> Neighbor (Standard A* behavior if LOS fails)
                if (!loS)
                {
                    float newCost = costSoFar[current] + (current - neighbor).cast<float>().norm();
                    if (costSoFar.find(neighbor) == costSoFar.end() ||
                        newCost < costSoFar[neighbor])
                    {
                        costSoFar[neighbor] = newCost;
                        float priority = newCost + heuristic(neighbor, end);
                        frontier.push({priority, neighbor});
                        cameFrom[neighbor] = current;
                    }
                }
            }
        }

        std::vector<Eigen::Vector2i> path;

        if (cameFrom.find(end) == cameFrom.end())
        {
            return path;
        }

        Eigen::Vector2i cur = end;
        while (cur != start)
        {
            path.push_back(cur);
            // In Theta*, this jumps across many nodes at once
            cur = cameFrom[cur];
        }
        path.push_back(start);

        std::reverse(path.begin(), path.end());
        return path;
    }

    Eigen::Vector2i ConvertWorldToGrid(Eigen::Vector2f worldPos)
    {
        return Eigen::Vector2i((int)(((worldPos.x() + fieldSetup.fieldRealSizeX / 2.0f) / fieldSetup.fieldRealSizeX) * fieldSetup.gridSizeX), (int)(((worldPos.y() + fieldSetup.fieldRealSizeY / 2.0f) / fieldSetup.fieldRealSizeY) * fieldSetup.gridSizeY));
    }

    Eigen::Vector2f ConvertGridToWorld(Eigen::Vector2f gridPos)
    {
        return Eigen::Vector2f(
            (gridPos.x() / (float)fieldSetup.gridSizeX) * fieldSetup.fieldRealSizeX -
                fieldSetup.fieldRealSizeX / 2.0f,
            (gridPos.y() / (float)fieldSetup.gridSizeY) * fieldSetup.fieldRealSizeY -
                fieldSetup.fieldRealSizeY / 2.0f);
    }

private:
    // Bresenham's Line Algorithm for Line of Sight
    bool lineOfSight(Eigen::Vector2i s, Eigen::Vector2i s_prime)
    {
        int x0 = s.x();
        int y0 = s.y();
        int x1 = s_prime.x();
        int y1 = s_prime.y();

        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        while (true)
        {
            if (getWorkingGrid(x0, y0) != std::byte{0})
            {
                return false;
            }

            if (x0 == x1 && y0 == y1) break;

            int e2 = 2 * err;
            if (e2 > -dy)
            {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                y0 += sy;
            }
        }

        return true;
    }

    float EuclideanDistance(int x1, int y1, int x2, int y2)
    {
        return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
    }

    float heuristic(Eigen::Vector2i current, Eigen::Vector2i goal)
    {
        return (current - goal).cast<float>().norm();
    }

    void DrawCircle(Eigen::Vector2i pos, int nodeRadius, std::byte* grid)
    {
        int r2 = nodeRadius * nodeRadius;
        int posX = pos.x();
        int posY = pos.y();

        for (int x = -nodeRadius; x <= nodeRadius; x++)
        {
            for (int y = -nodeRadius; y <= nodeRadius; y++)
            {
                if (x * x + y * y <= r2)
                {
                    int targetX = posX + x;
                    int targetY = posY + y;

                    if (targetX >= 0 && targetX < fieldSetup.gridSizeX && targetY >= 0 &&
                        targetY < fieldSetup.gridSizeY)
                    {
                        grid[targetX + targetY * fieldSetup.gridSizeX] = std::byte{3};
                    }
                }
            }
        }
    }

    void createObstacles()
    {
        for (const auto& bound : fieldSetup.obstacleBounds)
        {
            Eigen::Vector2i minGrid =
                ConvertWorldToGrid(Eigen::Vector2f(bound.first.x(), bound.first.y()));
            Eigen::Vector2i maxGrid =
                ConvertWorldToGrid(Eigen::Vector2f(bound.second.x(), bound.second.y()));

            for (int i = minGrid.x(); i <= maxGrid.x(); i++)
            {
                for (int j = minGrid.y(); j <= maxGrid.y(); j++)
                {
                    setObstacleGrid(i, j, std::byte{1});
                }
            }
        }
    }

    void growObstacles(float growDistance)
    {
        for (int x = 0; x < fieldSetup.gridSizeX; x++)
        {
            for (int y = 0; y < fieldSetup.gridSizeY; y++)
            {
                if (getObstacleGrid(x, y) == std::byte{1})
                {
                    int minX = x - growDistance;
                    int maxX = x + growDistance + 1;
                    int minY = y - growDistance;
                    int maxY = y + growDistance + 1;

                    for (int i = minX; i < maxX; i++)
                    {
                        for (int j = minY; j < maxY; j++)
                        {
                            if (i >= 0 && i < fieldSetup.gridSizeX && j >= 0 &&
                                j < fieldSetup.gridSizeY)
                            {
                                if (getObstacleGrid(i, j) == std::byte{0})
                                {
                                    if (EuclideanDistance(x, y, i, j) <= growDistance)
                                    {
                                        setObstacleGrid(i, j, std::byte{2});
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    void growEdges(float growDistance)
    {
        int growSize = growDistance - 1;

        for (int i = 0; i <= growSize; i++)
        {
            for (int j = 0; j < fieldSetup.gridSizeX; j++)
            {
                if (i < fieldSetup.gridSizeY)
                {
                    setObstacleGrid(j, i, std::byte{2});
                    setObstacleGrid(j, fieldSetup.gridSizeY - i - 1, std::byte{2});
                }
            }

            for (int j = 0; j < fieldSetup.gridSizeY; j++)
            {
                if (i < fieldSetup.gridSizeX)
                {
                    setObstacleGrid(i, j, std::byte{2});
                    setObstacleGrid(fieldSetup.gridSizeX - i - 1, j, std::byte{2});
                }
            }
        }
    }
};

const FieldSetup ThetaStar::fieldSetup = FieldSetup::ARC_ADJUSTED();

}  // namespace src::AutoPathing

#endif