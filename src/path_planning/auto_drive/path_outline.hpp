#include <vector>

#include <eigen3/Eigen/Dense>

class PathOutline
{
public:
    float MAX_POS_ERROR = 0.18f;

    PathOutline(std::vector<Eigen::Vector2f> outline, bool repeats)
        : outline(outline),
          repeats(repeats)
    {
    }

    PathOutline() : outline({}), repeats(false) {}

    Eigen::Vector2f evaluatePath(Eigen::Vector2f currentRobotWorldPos)
    {
        Eigen::Vector2f goal = outline[pathIndex];
        float distanceToGoal = (goal - currentRobotWorldPos).norm();

        if (distanceToGoal <= MAX_POS_ERROR)
        {
            pathIndex++;
            if (pathIndex >= outline.size())
            {
                if (repeats)
                {
                    pathIndex = 0;
                }
                else
                {
                    pathIndex = outline.size() - 1;
                    pathFinished = true;
                }
            }

            goal = outline[pathIndex];
        }

        return goal;
    }

    void startPath()
    {
        pathIndex = 0;
        pathFinished = false;
    }

    bool isPathFinished() { return pathFinished; }

    int pointCount() { return outline.size(); }

private:
    int pathIndex = 0;
    bool repeats = false;
    bool pathFinished = false;

    std::vector<Eigen::Vector2f> outline;
};