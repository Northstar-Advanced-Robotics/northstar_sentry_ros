#ifndef CUBIC_BEZIER_HPP
#define CUBIC_BEZIER_HPP

#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>

namespace src::AutoPathing {

class CubicBezier
{
public:
    static const int LENGTH_CALC_SAMPLES = 16;

    Eigen::Vector2f start;
    Eigen::Vector2f end;
    Eigen::Vector2f controlStart;
    Eigen::Vector2f controlEnd;

    float length;

    // Default constructor
    CubicBezier() 
        : start(0,0), end(0,0), controlStart(0,0), controlEnd(0,0), length(0) 
    {}
    CubicBezier(Eigen::Vector2f start_, Eigen::Vector2f end_, Eigen::Vector2f controlStart_, Eigen::Vector2f controlEnd_)
    {
        start = start_;
        end = end_;
        controlStart = controlStart_;
        controlEnd = controlEnd_;

        length = CalculateLength(LENGTH_CALC_SAMPLES);
    }

    Eigen::Vector2f Evaluate(float t) const
    {
        float oneMinusT = 1.0f - t;
        float t2 = t * t;
        float omt2 = oneMinusT * oneMinusT;

        return start * (oneMinusT * omt2) + 
               controlStart * (3.0f * omt2 * t) +  
               controlEnd * (3.0f * oneMinusT * t2) + 
               end * (t * t2);
    }

    Eigen::Vector2f EvaluateDerivative(float t) const
    {
        float oneMinusT = 1.0f - t;
        return (3.0f * (oneMinusT * oneMinusT) * (controlStart - start)) + 
               (6.0f * oneMinusT * t * (controlEnd - controlStart)) + 
               (3.0f * (t * t) * (end - controlEnd));
    }

    Eigen::Vector2f EvaluateSecondDerivative(float t) const
    {
        return (6.0f * (1.0f - t) * (controlEnd - 2.0f * controlStart + start)) + 
               (6.0f * t * (end - 2.0f * controlEnd + controlStart));
    }

    float CalculateLength(int samples)
    {
        float totalLength = 0.0f;
        Eigen::Vector2f prevPoint = start;

        for (int i = 0; i < samples; i++)
        {
            Eigen::Vector2f point = Evaluate((float)(i + 1) / samples);
            totalLength += (prevPoint - point).norm();
            prevPoint = point;
        }

        return totalLength;
    }
};

}

#endif