#include <vector>

#include <eigen3/Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

class PathOutline
{
public:
    const float MAX_POS_ERROR = 0.05f;

    PathOutline(std::vector<Eigen::Vector2f> outline, bool repeats) : outline(outline), repeats(repeats) { }

    Eigen::Vector2f evaluatePath(Eigen::Vector2f currentRobotWorldPos)
    {
        Eigen::Vector2f goal = outline[pathIndex];
        float distanceToGoal = (goal - currentRobotWorldPos).norm();

        if (distanceToGoal <= MAX_POS_ERROR)
        {
            pathIndex++; 
            if (pathIndex >= outline.size()) 
            { 
                if (repeats) { pathIndex = 0; }
                else { pathIndex = outline.size() - 1; }
            }

            goal = outline[pathIndex];
        }
        
        return goal;
    }

private:
    int pathIndex = 0;
    bool repeats = false;

    std::vector<Eigen::Vector2f> outline;
};