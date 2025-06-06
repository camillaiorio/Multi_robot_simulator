#pragma once
#include "grid.h"
#include "draw_helpers.h"

using Affine2f = Eigen::Transform<float, 2, Eigen::Affine>;

struct GridMapping {
  Eigen::Affine2f _w2g, _g2w;

  inline float resolution() const {
    return _g2w.linear()(0,0);
  }

  inline const Vector2f origin() const {
    return _g2w.translation();
  }

  void reset(const Vector2f grid_origin, float res) {
    _g2w.setIdentity();
    _g2w.translation() = grid_origin;
    _g2w.linear() <<
      res, 0,
      0, -res;
    _w2g = _g2w.inverse();
  }

  inline Vector2f world2grid(const Vector2f& src) const {
    return _w2g*src;
  };

  inline Vector2f grid2world(const Vector2f& src) const {
    return _g2w*src;
  }

  inline Eigen::Matrix2f world2gridDerivative() const {
    return _w2g.linear();
  };

  
};

// grid mapping class
struct GridMap: public Grid_<uint8_t>, public GridMapping{

  inline const Vector2f center() const {
    return grid2world(Vector2f(cols/2, rows/2));
  }
  
  
  // loads a map from an image
  void loadFromImage(const char* filename, float resolution);

  GridMap(float resolution, int rows=0, int cols=0);

 
  bool scanRay(Vector2f& hit, const Vector2f& origin, const Vector2f& direction,
               const float max_range) const;

  float scanRay(const Vector2f& origin,
                        const Vector2f& direction,
                 const float max_range) const;
};
