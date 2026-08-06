#ifndef CUBIC_BEZIER_FITTER_HPP
#define CUBIC_BEZIER_FITTER_HPP

#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>

#include "cubic_bezier.hpp"

namespace src::AutoPathing {

class CubicBezierFitter
{
public:
    static CubicBezier FitCubicMonotonic(
    const std::vector<Eigen::Vector2f>& pts,
    int maxIters = 128,
    float tol = 1e-6f)
{
    if (pts.size() < 2) {
        return CubicBezier();
    }

    const int n = (int)pts.size();

    // Fixed endpoints
    Eigen::Vector2f P0 = pts.front();
    Eigen::Vector2f P3 = pts.back();

    // Build monotonic coordinate frame
    Eigen::Vector2f D = (P3 - P0);
    float len = D.norm();
    if (len < 1e-8f) {
        return CubicBezier(P0, P3, P0, P3);
    }
    D /= len;

    Eigen::Vector2f N(-D.y(), D.x());

    // --- Chord-length parameterization (fixed) ---
    std::vector<float> t(n);
    t[0] = 0.0f;
    float totalDist = 0.0f;
    for (int i = 1; i < n; i++) {
        totalDist += (pts[i] - pts[i - 1]).norm();
    }

    if (totalDist <= 1e-8f) {
        for (int i = 0; i < n; i++) {
            t[i] = float(i) / float(n - 1);
        }
    } else {
        float acc = 0.0f;
        for (int i = 1; i < n; i++) {
            acc += (pts[i] - pts[i - 1]).norm();
            t[i] = acc / totalDist;
        }
    }

    // --- Control points parameterized in monotonic frame ---
    // P1 = P0 + a * D + b * N
    // P2 = P3 - c * D + d * N
    float a = len / 3.0f;
    float c = len / 3.0f;
    float b = 0.0f;
    float d = 0.0f;

    auto evalError = [&](float a, float b, float c, float d)
    {
        Eigen::Vector2f P1 = P0 + a * D + b * N;
        Eigen::Vector2f P2 = P3 - c * D + d * N;
        return ComputeError(pts, t, P0, P1, P2, P3);
    };

    float prevError = evalError(a, b, c, d);

    for (int iter = 0; iter < maxIters; iter++)
    {
        Eigen::Matrix4f JTJ = Eigen::Matrix4f::Zero();
        Eigen::Vector4f JTr = Eigen::Vector4f::Zero();

        // Build Jacobian in (a, b, c, d) space
        for (int i = 0; i < n; i++)
{
    float ti = t[i];
    float u  = 1.0f - ti;

    float B1 = 3.0f * u * u * ti;
    float B2 = 3.0f * u * ti * ti;

    Eigen::Vector2f P1 = P0 + a * D + b * N;
    Eigen::Vector2f P2 = P3 - c * D + d * N;

    Eigen::Vector2f B =
        u*u*u * P0 +
        B1    * P1 +
        B2    * P2 +
        ti*ti*ti * P3;

    Eigen::Vector2f r = B - pts[i];

    Eigen::Vector2f dB_da = B1 * D;
    Eigen::Vector2f dB_db = B1 * N;
    Eigen::Vector2f dB_dc = -B2 * D;
    Eigen::Vector2f dB_dd = B2 * N;

    Eigen::Vector2f J[4] = {
        dB_da, dB_db, dB_dc, dB_dd
    };

    // Accumulate JᵀJ and Jᵀr
    for (int m = 0; m < 4; m++)
{
    float JmN = J[m].dot(N);

    for (int n = 0; n < 4; n++)
    {
        float JnN = J[n].dot(N);
        JTJ(m, n) += J[m].dot(J[n]);
        JTJ(m, n) += JmN * JnN;
    }

    float e = r.dot(N);
    JTr(m) += JmN * e;
}
}


        Eigen::Vector4f delta = JTJ.ldlt().solve(-JTr);
        if (!delta.allFinite()) break;

        float na = std::max(1e-6f, a + delta[0]);
        float nb = b + delta[1];
        float nc = std::max(1e-6f, c + delta[2]);
        float nd = d + delta[3];

        // Enforce monotonic curvature (same side of chord)
        if (nb * nd < 0.0f) {
            nd = nb;
        }

        float newError = evalError(na, nb, nc, nd);

        if (newError < prevError) {
            a = na; b = nb; c = nc; d = nd;
            if (std::abs(prevError - newError) / prevError < tol) {
                break;
            }
            prevError = newError;
        } else {
            break;
        }
    }

    Eigen::Vector2f P1 = P0 + a * D + b * N;
    Eigen::Vector2f P2 = P3 - c * D + d * N;

    return CubicBezier(P0, P3, P1, P2);
}


    static CubicBezier FitCubic(const std::vector<Eigen::Vector2f>& pts, int maxIters = 100, int reparamEvery = 1, float tol = 1e-5f)
    {
        if (pts.size() < 2) { return CubicBezier(); }

        // Convert Vector2i to Vector2f for calculation
        // std::vector<Eigen::Vector2f> pts;
        // pts.reserve(pointsInt.size());
        // for (const auto& pi : pointsInt) {
        //     pts.push_back(pi.cast<float>());
        // }

        Eigen::Vector2f P0 = pts.front();
        Eigen::Vector2f P3 = pts.back();
        int n = (int)pts.size();

        // --- Initial Chord-Length Parameterization ---
        std::vector<float> t(n);
        t[0] = 0.0f;
        float totalDist = 0.0f;
        
        // Calculate total length
        for (int i = 1; i < n; i++) {
            totalDist += (pts[i] - pts[i - 1]).norm();
        }

        if (totalDist <= 0.0f) {
            for (int i = 0; i < n; i++) t[i] = (float)i / (n - 1);
        } else {
            float acc = 0.0f;
            for (int i = 1; i < n; i++) {
                acc += (pts[i] - pts[i - 1]).norm();
                t[i] = acc / totalDist;
            }
        }

        // --- Initial Guess (Heuristic) ---
        Eigen::Vector2f P1 = P0 + (P3 - P0) * (1.0f / 3.0f);
        Eigen::Vector2f P2 = P0 + (P3 - P0) * (2.0f / 3.0f);

        // --- Levenberg-Marquardt Params ---
        float lambda = 1e-3f;
        float nu = 2.0f;

        float prevError = ComputeError(pts, t, P0, P1, P2, P3);

        for (int iter = 0; iter < maxIters; iter++)
        {
            // Build linear system components
            Eigen::Matrix4f JTJ = Eigen::Matrix4f::Zero();
            Eigen::Vector4f JTr = Eigen::Vector4f::Zero();

            BuildJTJAndJTr(pts, t, P0, P1, P2, P3, JTJ, JTr);

            // Add Lambda to Diagonal (Levenberg-Marquardt damping)
            Eigen::Matrix4f JTJ_lm = JTJ;
            for (int d = 0; d < 4; d++) JTJ_lm(d, d) += lambda;

            // Solve JTJ * delta = -JTr
            // Using LDLT decomposition is efficient for symmetric positive-definite matrices
            Eigen::Vector4f rhs = -JTr;
            Eigen::Vector4f delta = JTJ_lm.ldlt().solve(rhs);

            // Check if solution is valid (not NaN/Inf)
            if (!delta.allFinite()) {
                delta.setZero();
            }

            // Tentative Update
            Eigen::Vector2f newP1(P1.x() + delta[0], P1.y() + delta[1]);
            Eigen::Vector2f newP2(P2.x() + delta[2], P2.y() + delta[3]);

            float newError = ComputeError(pts, t, P0, newP1, newP2, P3);

            float currentError = prevError;

            if (newError < prevError) 
            {
                // Success: Accept step, decrease lambda
                P1 = newP1;
                P2 = newP2;
                prevError = newError;
                lambda *= std::max(1.0f / 3.0f, 1.0f - std::pow(2.0f, -(float)iter));
                nu = 2.0f;
            }
            else 
            {
                // Reject step, increase lambda
                lambda *= nu;
                nu *= 2.0f;
            }

            // Re-parameterize t values occasionally
            if (reparamEvery > 0 && (iter % reparamEvery == 0)) {
                Reparameterize(pts, t, P0, P1, P2, P3);
            }

            // Check convergence
            if (newError < currentError) {
                float relativeChange = std::abs(currentError - newError) / std::max(1e-12f, currentError);
                if (relativeChange < tol){
                    std::cout << "Converged after " << iter + 1 << " iterations." << std::endl;
                    break;
                } 
            }
        }

        return CubicBezier(P0, P3, P1, P2);
    }


    // use length to determen dencity
    // static std::vector<Eigen::Vector2f> PopulatePoints(std::vector<Eigen::Vector2i> pathInt, float spacing)
    // {
    //     // Convert Vector2i to Vector2f
    //     std::vector<Eigen::Vector2f> path;
    //     path.reserve(pathInt.size());
    //     for (const auto& pi : pathInt) {
    //         path.push_back(pi.cast<float>());
    //     }

    //     std::vector<Eigen::Vector2f> populatedPoints;
    //     if (path.size() < 2) return populatedPoints;

    //     for (size_t i = 0; i < path.size() - 1; i++)
    //     {
    //         Eigen::Vector2f start = path[i];
    //         Eigen::Vector2f end = path[i + 1];            
    //         Eigen::Vector2f segment = end - start;
    //         float segmentLength = 100.0f/segment.norm();
    //         int numPoints = std::max(15, (int)(segmentLength / spacing));

    //         for (int j = 0; j < numPoints; j++)
    //         {
    //             float t = (float)j / numPoints;
    //             populatedPoints.push_back(start + segment * t);
    //         }
    //     }
    //     // Ensure the last point is included
    //     populatedPoints.push_back(path.back());

    //     return populatedPoints;
    // }


    // Using angle to determine density
static std::vector<Eigen::Vector2f> PopulatePoints(const std::vector<Eigen::Vector2i>& pathInt, float spacing)
    {
        // 1. Convert to Float
        std::vector<Eigen::Vector2f> path;
        path.reserve(pathInt.size());
        for (const auto& pi : pathInt) {
            path.push_back(pi.cast<float>());
        }

        std::vector<Eigen::Vector2f> populatedPoints;
        if (path.size() < 2) return populatedPoints;

        // Helper: Calculate Angle Change (0 to PI) at a vertex index
        // 0 = Straight, PI = U-Turn
        auto getTurnAngle = [&](size_t idx) -> float {
            if (idx == 0 || idx >= path.size() - 1) return 0.0f; // Endpoints have no turn
            
            Eigen::Vector2f prev = (path[idx] - path[idx - 1]);
            Eigen::Vector2f next = (path[idx + 1] - path[idx]);
            
            if (prev.norm() < 1e-4f || next.norm() < 1e-4f) return 0.0f; // Safe check
            
            prev.normalize();
            next.normalize();
            
            // Dot product: 1.0 = Straight, 0.0 = 90 deg, -1.0 = 180 deg
            float dot = prev.dot(next);
            // We want the angle of CHANGE. 
            // If dot is 1 (straight), change is 0. 
            // If dot is 0 (90 deg), change is PI/2.
            return std::acos(std::clamp(dot, -1.0f, 1.0f)); 
        };

        // 2. Iterate Segments
        for (size_t i = 0; i < path.size() - 1; i++)
        {
            Eigen::Vector2f start = path[i];
            Eigen::Vector2f end = path[i + 1];            
            Eigen::Vector2f segment = end - start;
            float segmentLength = segment.norm();

            // Collect all 't' values (0.0 to 1.0) where we want a point
            std::vector<float> t_values;

            // A. Base Uniform Distribution (Keep basic coverage)
            int basePoints = std::max(2, (int)(segmentLength / spacing));
            for (int j = 0; j < basePoints; j++) {
                t_values.push_back((float)j / basePoints);
            }

            // B. Angle-Based Concentration
            float angleStart = getTurnAngle(i);     // Turn at the Start of this segment
            float angleEnd   = getTurnAngle(i + 1); // Turn at the End of this segment

            // Threshold: Consider anything > 10 degrees (0.17 rad) a "turn" worth reinforcing
            float turnThreshold = 0.17f; 
            
            // If Start is a corner, add dense points in the first 15% of the segment
            if (angleStart > turnThreshold) {
                int denseCount = 6; // Add 6 extra points near the corner
                float range = 0.40f; 
                for(int k = 1; k <= denseCount; k++) {
                    t_values.push_back(((float)k / (denseCount + 1)) * range);
                }
            }

            // If End is a corner, add dense points in the last 15% of the segment
            if (angleEnd > turnThreshold) {
                int denseCount = 6; 
                float range = 0.40f; 
                for(int k = 1; k <= denseCount; k++) {
                    t_values.push_back(1.0f - (((float)k / (denseCount + 1)) * range));
                }
            }

            // C. Sort and Unique to clean up the list
            std::sort(t_values.begin(), t_values.end());
            // Remove duplicates (optional, but good for cleanliness)
            auto last = std::unique(t_values.begin(), t_values.end());
            t_values.erase(last, t_values.end());

            // D. Generate Actual Points
            for (float t : t_values) {
                populatedPoints.push_back(start + segment * t);
            }
        }
        
        // Ensure the very last point of the path is included
        populatedPoints.push_back(path.back());

        return populatedPoints;
    }

static std::vector<Eigen::Vector2f>
PopulatePointsNew(const std::vector<Eigen::Vector2i>& pathInt)
{
    const float POINTS_PER_DIST = 18.0f; // in grid space
    const float POINTS_PER_RAD = 1.0f;

    if (pathInt.size() < 2)
        return {};

    std::vector<Eigen::Vector2f> path;
    path.reserve(pathInt.size());
    for (const auto& pi : pathInt)
        path.push_back(pi.cast<float>());

    std::vector<Eigen::Vector2f> newPath;
    newPath.reserve(path.size() * 2);

    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        Eigen::Vector2f a = path[i];
        Eigen::Vector2f b = path[i + 1];

        float d = (b - a).norm();
        int distancePoints = std::max(2, int(d / POINTS_PER_DIST) + 1);
        int anglePoints = i == 0 ? 0 : std::abs(angle(path[ i -1], path[i], path[ i + 1])) * POINTS_PER_RAD;

        int points = std::max(distancePoints, anglePoints);

        newPath.push_back(a);

        for (int j = 1; j < points - 1; ++j)
        {
            float t = float(j) / float(points - 1);
            newPath.push_back(lerp(a, b, t));
        }
    }

    newPath.push_back(path.back());
    return newPath;
}

private:

    // Static Math Helper to avoid instantiating CubicBezier inside tight loops
    static Eigen::Vector2f BezierPoint(float t, const Eigen::Vector2f& P0, const Eigen::Vector2f& P1, const Eigen::Vector2f& P2, const Eigen::Vector2f& P3)
    {
        float u = 1.0f - t;
        float tt = t * t;
        float uu = u * u;
        float uuu = uu * u;
        float ttt = tt * t;

        return (uuu * P0) + (3 * uu * t * P1) + (3 * u * tt * P2) + (ttt * P3);
    }

    static Eigen::Vector2f BezierDerivative(float t, const Eigen::Vector2f& P0, const Eigen::Vector2f& P1, const Eigen::Vector2f& P2, const Eigen::Vector2f& P3)
    {
        float u = 1.0f - t;
        return (3.0f * u * u * (P1 - P0)) + 
               (6.0f * u * t * (P2 - P1)) + 
               (3.0f * t * t * (P3 - P2));
    }

    static Eigen::Vector2f BezierSecondDerivative(float t, const Eigen::Vector2f& P0, const Eigen::Vector2f& P1, const Eigen::Vector2f& P2, const Eigen::Vector2f& P3)
    {
        return (6.0f * (1.0f - t) * (P2 - 2.0f * P1 + P0)) + 
               (6.0f * t * (P3 - 2.0f * P2 + P1));
    }

    static float ComputeError(const std::vector<Eigen::Vector2f>& pts, const std::vector<float>& t, 
                              const Eigen::Vector2f& P0, const Eigen::Vector2f& P1, const Eigen::Vector2f& P2, const Eigen::Vector2f& P3)
    {
        float sum = 0.0f;
        for (size_t i = 0; i < pts.size(); i++)
        {
            Eigen::Vector2f B = BezierPoint(t[i], P0, P1, P2, P3);
            sum += (B - pts[i]).squaredNorm();
        }
        return sum;
    }

    static void BuildJTJAndJTr(const std::vector<Eigen::Vector2f>& pts, const std::vector<float>& t, 
                               const Eigen::Vector2f& P0, const Eigen::Vector2f& P1, const Eigen::Vector2f& P2, const Eigen::Vector2f& P3,
                               Eigen::Matrix4f& JTJ, Eigen::Vector4f& JTr)
    {
        // P1.x, P1.y, P2.x, P2.y
        for (size_t i = 0; i < pts.size(); i++)
        {
            float ti = t[i];
            float u = 1.0f - ti;

            // Bernstein coefficients for P1 and P2
            // P(t) = b0*P0 + b1*P1 + b2*P2 + b3*P3
            // dP/dP1 = b1, dP/dP2 = b2
            float b1 = 3.0f * u * u * ti;
            float b2 = 3.0f * u * ti * ti;

            Eigen::Vector2f B = BezierPoint(ti, P0, P1, P2, P3);
            Eigen::Vector2f r = B - pts[i]; 

            // Jacobian Structure:
            // J_i = [ b1*I2, b2*I2 ] where I2 is 2x2 identity
            // This decomposes into Jx rows and Jy rows.

            // Constructing 4x1 Jacobian vectors for x and y components
            Eigen::Vector4f Jx; Jx << b1, 0, b2, 0;
            Eigen::Vector4f Jy; Jy << 0, b1, 0, b2;

            // JTJ accumulation: J^T * J
            // For a single point residual vector r_i = [rx, ry], the term is J_i^T * J_i
            JTJ += Jx * Jx.transpose() + Jy * Jy.transpose();

            // JTr accumulation: J^T * r
            // JTr += Jx * rx + Jy * ry
            JTr += Jx * r.x() + Jy * r.y();
        }

        // Gradient of sum of squares is 2 * J^T * r
        JTr *= 2.0f;
        JTJ *= 2.0f; // Hessian approximation
    }

    static void Reparameterize(const std::vector<Eigen::Vector2f>& pts, std::vector<float>& t, 
                               const Eigen::Vector2f& P0, const Eigen::Vector2f& P1, const Eigen::Vector2f& P2, const Eigen::Vector2f& P3)
    {
        for (size_t i = 0; i < pts.size(); i++) {
            t[i] = ProjectPointOntoBezier(pts[i], t[i], P0, P1, P2, P3);
        }
    }

    static float ProjectPointOntoBezier(const Eigen::Vector2f& Q, float t0, 
                                        const Eigen::Vector2f& P0, const Eigen::Vector2f& P1, const Eigen::Vector2f& P2, const Eigen::Vector2f& P3, 
                                        int newtonIters = 4)
    {
        float t = std::clamp(t0, 0.0f, 1.0f);

        for (int k = 0; k < newtonIters; k++)
        {
            Eigen::Vector2f C = BezierPoint(t, P0, P1, P2, P3);
            Eigen::Vector2f dC = BezierDerivative(t, P0, P1, P2, P3);
            Eigen::Vector2f ddC = BezierSecondDerivative(t, P0, P1, P2, P3);

            Eigen::Vector2f diff = C - Q;
            float numerator = diff.dot(dC);
            float denominator = dC.dot(dC) + diff.dot(ddC);

            if (std::abs(denominator) < 1e-12f) break;

            float dt = numerator / denominator;
            t -= dt;

            if (t <= 0.0f) { t = 0.0f; break; }
            if (t >= 1.0f) { t = 1.0f; break; }
            if (std::abs(dt) < 1e-8f) break;
        }
        return t;
    }

    static Eigen::Vector2f lerp(Eigen::Vector2f a, Eigen::Vector2f b, float t)
    {
        return (1 - t) * a + t * b;
    }

    static float angle(Eigen::Vector2f a, Eigen::Vector2f b, Eigen::Vector2f c)
    {
        return std::atan2(a.y() - b.y(), a.x() - b.x()) - std::atan2(c.y() - b.y(), c.x() - b.x());
    }

public:
    static int findFirstConcavityFlip(std::vector<Eigen::Vector2i> pts)
    {
        if (pts.size() < 3)
            return pts.size();

        int prevSign = 0;

        for (int i = 1; i < pts.size() - 1; i++)
        {
            // Integer differences
            float ax = pts[i].x() - pts[i - 1].x();
            float ay = pts[i].y() - pts[i - 1].y();
            float bx = pts[i + 1].x() - pts[i].x();
            float by = pts[i + 1].y() - pts[i].y();

            // Integer 2D cross product
            float cross = ax * by - ay * bx;

            int sign = cross == 0 ? 0 : (cross > 0 ? 1 : -1);

            if (sign == 0)
                continue; // straight, ignore

            if (prevSign == 0)
            {
                prevSign = sign; // this is our baseline curvature direction
            }
            else if (sign != prevSign)
            {
                // curvature changed sign → first concavity flip
                return i+1;
            }
        }

        // No flip → return end as expected
        return pts.size();
    }
};

}

#endif