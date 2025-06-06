#include "draw_helpers.h"

void drawLine(Canvas& dest, const Eigen::Vector2f& p0, const Eigen::Vector2f& p1, uint8_t color) {
  cv::line(dest, cv::Point(p0[0], p0[1]), cv::Point(p1[0], p1[1]),
           cv::Scalar(color, color, color), 1);
}

void drawCircle(Canvas& dest, const Eigen::Vector2f& center, int radius, uint8_t color) {
  cv::circle(dest, cv::Point(center[0], center[1]), radius,
             cv::Scalar(color, color, color));
}

int showCanvas(Canvas& canvas, int timeout_ms) {
  cv::Mat resized;
  float scale = 0.5; // or any factor < 1 to reduce size
  cv::resize(canvas, resized, cv::Size(), scale, scale, cv::INTER_NEAREST);
  cv::imshow("canvas", resized);
  int key = cv::waitKey(timeout_ms);
  if (key == 27)  // exit on ESC
    exit(0);
  return key;
}