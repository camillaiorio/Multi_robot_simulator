#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

using Canvas = cv::Mat;

// draws a line from p0 to p1
void drawLine(Canvas& dest, const Eigen::Vector2f& p0, const Eigen::Vector2f& p1, uint8_t color);
void drawCircle(Canvas& dest, const Eigen::Vector2f& center, int radius, uint8_t color);
int showCanvas(Canvas& canvas, int timeout_ms);
