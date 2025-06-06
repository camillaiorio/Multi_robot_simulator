#include "grid_map.h"
using namespace std;

GridMap::GridMap(float resolution_, int rows_, int cols_):
  Grid_<uint8_t>(rows_, cols_){
  reset(Vector2f(0,0), resolution_);
}

float GridMap::scanRay(const Vector2f& origin,
                        const Vector2f& direction,
                        const float max_range) const {
  float range = 0;

  while (range < max_range) {
    Vector2i int_endpoint=world2grid(origin + direction * range).cast<int>();
    
    if (!inside(int_endpoint))
      return max_range;

    if ((*this)(int_endpoint) < 127)
      return range;

    range +=resolution();
  }
  
  return max_range;
}

void GridMap::loadFromImage(const char* filename, float res) {
    cerr << "Loading map file from: [" << filename << "]" << endl;
  cv::Mat m = cv::imread(filename);
  if (m.rows == 0) {
    throw std::runtime_error("unable to load image");
  }
  cv::Mat loaded_image;
  cv::cvtColor(m, loaded_image, cv::COLOR_BGR2GRAY);
  resize(loaded_image.rows, loaded_image.cols);
  reset(Vector2f(0,0), res);
  memcpy(&cells[0], loaded_image.data, cells.size());
}
