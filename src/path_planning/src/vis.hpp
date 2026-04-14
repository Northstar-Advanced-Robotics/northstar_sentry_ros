#ifndef MAP_VISUALIZER_HPP
#define MAP_VISUALIZER_HPP

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cmath>

#include <eigen3/Eigen/Dense>

#include "auto_drive/a_star.hpp" 
#include "auto_drive/cubic_bezier.hpp" 

namespace src::viz {

class MapVisualizer {
public:
    // Change: We pass the base filename, e.g., "map_output"
    static void SavePPM(const std::string& baseFilename, 
                        const src::astar::AStar& mapData, 
                        const std::vector<Eigen::Vector2i>& path,
                        const std::vector<src::astar::CubicBezier>& bezierPath = {}) 
    {
        // STATIC COUNTER: This persists between function calls
        static int frameCount = 0; 

        // Construct unique filename: "map_output_0.ppm", "map_output_1.ppm", etc.
        std::string filename = baseFilename + "_" + std::to_string(frameCount) + ".ppm";


        std::ofstream file(filename);
        if (!file) {
            // If this prints, you might have run out of disk space or have bad permissions
            std::cerr << "[Error] Visualizer could not create file: " << filename << std::endl;
            return;
        }

        int width = mapData.GetSizeX();
        int height = mapData.GetSizeY();

        // --- PRE-CALCULATION STEP ---
        std::vector<std::vector<bool>> bezierGrid(width, std::vector<bool>(height, false));
        std::vector<std::vector<bool>> controlPointGrid(width, std::vector<bool>(height, false));

        for (const auto& curve : bezierPath) {
            // FIX: Safety check for zero-length or garbage curves
            if (curve.length <= 0.001f || std::isnan(curve.length)) continue;

            float len = curve.length; 
            
            // FIX: Cap the samples to prevent overflow/huge loops
            int samples = (int)(len * 2.0f); 
            if (samples < 10) samples = 10;
            if (samples > 10000) samples = 10000; // Hard limit

            for (int i = 0; i <= samples; i++) {
                float t = (float)i / (float)samples;
                Eigen::Vector2f pt = curve.Evaluate(t);
                
                int bx = (int)(pt.x() + 0.5f);
                int by = (int)(pt.y() + 0.5f);

                if (bx >= 0 && bx < width && by >= 0 && by < height) {
                    bezierGrid[bx][by] = true;
                }
            }

            std::vector<Eigen::Vector2f> controls = {curve.controlStart, curve.controlEnd};
            for(auto& c : controls) {
                int cx = (int)(c.x() + 0.5f);
                int cy = (int)(c.y() + 0.5f);
                if (cx >= 0 && cx < width && cy >= 0 && cy < height) {
                    controlPointGrid[cx][cy] = true;
                }
            }
        }

        // --- WRITING STEP ---
        file << "P3\n" << width << " " << height << "\n255\n";

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                
                bool isPath = false;
                for (const auto& p : path) {
                    if (p.x() == x && p.y() == y) {
                        isPath = true;
                        break;
                    }
                }

                bool isStart = (!path.empty() && path.front().x() == x && path.front().y() == y);
                bool isEnd = (!path.empty() && path.back().x() == x && path.back().y() == y);
                int gridVal = mapData.GetGridValue(x, y);

                // Priority: ControlPoints > Bezier > Start/End > A* Path > Obstacle > Empty
                if (controlPointGrid[x][y]) {
                    file << "255 255 0 "; // Yellow
                }
                else if (bezierGrid[x][y]) {
                    file << "255 0 255 "; // Magenta
                }
                else if (isStart) {
                    file << "0 0 255 "; // Blue
                }
                else if (isEnd) {
                    file << "255 165 0 "; // Orange
                }
                else if (isPath) {
                    file << "0 255 0 "; // Green
                }
                else if (gridVal == 1) {
                    file << "255 0 0 "; // Red
                }
                else if (gridVal == 2) {
                    file << "128 128 128 "; // Grey
                }
                else {
                    file << "0 0 0 "; // Black
                }
            }
            file << "\n";
        }

        file.close();
        // Removed the cout so it doesn't spam your console
        // std::cout << "Saved " << filename << std::endl; 
    }
};

}

#endif