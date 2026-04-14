#ifndef CUBIC_BEZIER_FITTER_SMOOTH__HPP
#define CUBIC_BEZIER_FITTER_SMOOTH__HPP

#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <functional>
#include <iostream>

#include "cubic_bezier.hpp"

namespace src::astar {

class CubicBezierFitterSmooth
{
public:
    // Define a callback type for collision checking
    // Returns true if the point is SAFE (valid), false if OBSTACLE
    using CollisionCheck = std::function<bool(Eigen::Vector2i)>;

    /**
     * @brief Generates a smooth, collision-free path using Recursive Subdivision.
     * * @param rawPath The straight-line waypoints from Theta*
     * @param isValid Function that returns true if a grid cell is safe
     * @return std::vector<Eigen::Vector2f> A list of dense points representing the smooth curve
     */
    static std::vector<Eigen::Vector2f> SmoothPath(const std::vector<Eigen::Vector2i>& rawPath, CollisionCheck isValid)
    {
        std::vector<Eigen::Vector2f> smoothPoints;
        if (rawPath.size() < 2) return smoothPoints;

        // Add the very first point
        smoothPoints.push_back(rawPath[0].cast<float>());

        // Iterate through all segments of the raw path (A -> B -> C...)
        for (size_t i = 0; i < rawPath.size() - 1; ++i)
        {
            CubicBezier bezier;
            bezier.start = rawPath[i].cast<float>();
            bezier.end = rawPath[i+1].cast<float>();

            // Calculate Tangents for smooth transitions
            // Tangent at Start depends on previous point
            if (i == 0) bezier.controlStart = bezier.end - bezier.start; // First point: point towards next
            else bezier.controlStart = bezier.end - rawPath[i-1].cast<float>();

            // Tangent at End depends on next point
            if (i == rawPath.size() - 2) bezier.controlEnd = bezier.end - bezier.start; // Last point: point from prev
            else bezier.controlEnd = rawPath[i+2].cast<float>() - bezier.start;

            // normalize tangents
            if (bezier.controlStart.norm() > 1e-5) bezier.controlStart.normalize();
            if (bezier.controlEnd.norm() > 1e-5) bezier.controlEnd.normalize();

            // Run the Recursive Subdivision on this segment
            RecursiveSmooth(bezier, isValid, smoothPoints);
        }

        return smoothPoints;
    }

private:

    // The Recursive Core: Tries to fit a curve; if it hits a wall, splits the line and tries again.
    static void RecursiveSmooth(CubicBezier bezier, CollisionCheck isValid, 
                                std::vector<Eigen::Vector2f>& outPoints,
                                int depth = 0)
    {
        // Safety Break: Don't recurse infinitely if stuck
        if (depth > 8) {
            // Fallback: Just draw a straight line if we are too deep
            int straightSamples = (int)((bezier.end - bezier.start).norm() * 2.0f); 
            for (int i = 1; i <= straightSamples; i++) {
                 float t = (float)i / straightSamples;
                 outPoints.push_back(bezier.start + (bezier.end - bezier.start) * t);
            }
            return;
        }

        // 1. Generate Control Points (Heuristic: 1/3rd of distance)
        float dist = (bezier.end - bezier.start).norm();
        float controlDist = dist / 3.0f;
        
        bezier.controlStart = bezier.start + bezier.controlStart * controlDist;
        bezier.controlEnd = bezier.end - bezier.controlEnd * controlDist;

        // 2. Sample the proposed curve to check for collisions
        bool collision = false;
        int checkSamples = std::max(5, (int)(dist * 2.0f)); // Check every ~0.5 units
        
        for (int i = 1; i < checkSamples; ++i) {
            float t = (float)i / checkSamples;
            Eigen::Vector2f pt = bezier.Evaluate(t);
            
            // Check grid collision
            // We cast to int for the grid check, add 0.5 for rounding
            Eigen::Vector2i gridPt( (int)(pt.x() + 0.5f), (int)(pt.y() + 0.5f) );
            
            if (!isValid(gridPt)) {
                collision = true;
                break;
            }
        }

        // 3. Decision: Keep or Split
        if (!collision) {
            // SAFE: Add points to output
            // Use higher resolution for the final path than for the check
            int drawSamples = std::max(5, (int)(dist * 1.0f)); 
            for (int i = 1; i <= drawSamples; ++i) {
                float t = (float)i / drawSamples;
                outPoints.push_back(bezier.Evaluate(t));
            }
        } 
        else {
            // COLLISION: Split the straight segment in half and recurse
            Eigen::Vector2f midPoint = (bezier.start + bezier.end) * 0.5f;
            Eigen::Vector2f midTangent = (bezier.end - bezier.start).normalized(); // Tangent at mid is simply the direction of the line

            // Recurse Left (Start -> Mid)
            CubicBezier left;
            left.start = bezier.start;
            left.end = midPoint;
            left.controlStart = bezier.controlStart;
            left.controlEnd = midTangent;

            RecursiveSmooth(left, isValid, outPoints, depth + 1);
            
            // Create Right Object
            CubicBezier right;
            right.start = midPoint;
            right.end = bezier.end;
            right.controlStart = midTangent;
            right.controlEnd = bezier.controlEnd;

            RecursiveSmooth(right, isValid, outPoints, depth + 1);        
        }
    }
}; // namespace src::astar

}
#endif