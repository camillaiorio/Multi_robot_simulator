#pragma once
#include <vector>
#include <iostream>
#include <Eigen/Geometry>
#include <stdexcept>
#include <assert.h>
#include <limits>
#include "draw_helpers.h"

using Vector2i = Eigen::Matrix<int, 2, 1>;
using Vector2f = Eigen::Matrix<float, 2, 1>;
                               
template <typename CellType_>
struct Grid_{
  using CellType = CellType_;
  using CellTypeVector = std::vector<CellType_>;

  Grid_(int r=0, int c=0){
    resize(r,c);
  }

  void resize(int r, int c) {
    rows=r;
    cols=c;
    cells.resize(rows*cols);
  }
  
  inline CellType& operator()(int r, int c) {
    return cells[r*cols+c];
  }

  inline const CellType& operator()(int r, int c) const {
    return cells[r*cols+c];
  }

  inline bool inside(int r, int c) const{
    return r>=0 && r<rows && c>=0  && c<cols;
  }

  inline bool inside(const Vector2i& src) const {
    return src[0] >= 0 && src[0] < cols && src[1] >= 0 && src[1] < rows;
  }

  inline bool inside(const Vector2f& src) const {
    return src[0] >= 1 && src[0] < cols-1 && src[1] >= 1 && src[1] < rows-1;
  }

  inline CellType& operator()(const Vector2i& coordinates) {
    return (*this)(coordinates(1), coordinates(0));
  }

  inline const CellType& operator()(const Vector2i& coordinates) const {
    return (*this)(coordinates(1), coordinates(0));
  }

  inline const Vector2i ptr2idx(const CellType* cell) const {
    int pos =  cell - &cells[0];

    assert(pos>=0 && pos< cells.size());
    
    return Vector2i(pos%cols, pos/cols);
    
  }

  inline int distance2(const CellType* from, const CellType* to) const {
    return (ptr2idx(from)-ptr2idx(to)).squaredNorm();
  }
                                
  inline const CellType operator()(const Vector2f& coordinates) const {
    Vector2i c00, c01, c10, c11;
    c00  = coordinates.cast<int>();
    c10 = c00 + Vector2i(1,0);
    c01 = c00 + Vector2i(0,1);
    c11 = c00 + Vector2i(1,1);

    if (c00.x()>=cols-1 || c00.x()<0
        || c00.y()>=rows-1 || c00.y()<0) {
      std::cerr << c00.transpose() << " "<< c01.transpose() << std::endl;
      throw std::runtime_error("out of buonds");
    }
    
    const CellType& f00=(*this)(c00); // 11
    const CellType& f01=(*this)(c01); // 10
    const CellType& f10=(*this)(c10); // 01
    const CellType& f11=(*this)(c11); // 00
    
    float dx1 = coordinates.x() - c00.x();
    float dx2 = 1-dx1;
    float dy1 = coordinates.y() - c00.y();
    float dy2 = 1-dy1;
    return
      f00*(dx2*dy2)
      +f01*(dx2*dy1)
      +f10*(dx1*dy2)
      +f11*(dx1*dy1);

  }

  void rowDerivative(Grid_<CellType>& dest) const {
    dest.resize(rows, cols);
    for (int r=1; r<rows-1; ++r)
      for (int c=0; c<cols; ++c)
        dest(r,c)=(*this)(r+1,c)-(*this)(r-1,c);
  }

  void colDerivative(Grid_<CellType>& dest) const {
    dest.resize(rows, cols);
    for (int r=0; r<rows; ++r)
      for (int c=1; c<cols-1; ++c)
        dest(r,c)=(*this)(r,c+1)-(*this)(r,c-1);
  }

  template <typename  OtherCellType>
  Grid_<OtherCellType> cast() const {
    Grid_<OtherCellType> other(rows, cols);
    for (size_t i=0; i<cells.size(); ++i) {
      other.cells[i]=cells[i];
    }
    return other;
  }


  void fill(const CellType& src) {
    for(auto& d: cells)
      d=src;
  }
  
  void draw(Canvas& dest, bool equalize=false) const {
    using namespace std;
    CellType lower=0;
    CellType scale=1;
    if (equalize) {
      lower=std::numeric_limits<CellType>::max();
      CellType upper=std::numeric_limits<CellType>::min();
      for (const auto& v: cells) {
        lower=std::min(lower, v);
        upper=std::max(upper, v);
      }
      scale=255.f / (upper-lower);
    }

    dest = cv::Mat(rows, cols, CV_8UC1);
    uint8_t* dest_ptr=(uint8_t*) dest.data;
    for (size_t i=0; i<cells.size(); ++i)
      dest_ptr[i] = scale*(cells[i]-lower);
  }

  int rows;
  int cols;
  CellTypeVector cells;
};


template <typename CellType_>
std::ostream& operator<<(std::ostream& os, const Grid_<CellType_>& g) {
  for (int r=0; r<g.rows; ++r) {
    for (int c=0; c<g.cols; ++c) {
      os << g(r,c) << " ";
    }
    os << std::endl;
  }
  return os;
  
}

struct CT{
  using TransformType = Eigen::Transform<float, 2, Eigen::Affine>;
  TransformType _g2w=TransformType::Identity();
  TransformType _w2g=TransformType::Identity();

  void reset(const Eigen::Vector2f& grid_origin,
             const float res) {
    _g2w.linear() <<
      res, 0,
      0, -res;
    _g2w.translation()=grid_origin;
    _w2g=_g2w.inverse();
  }

  inline Eigen::Vector2f world2grid(const Eigen::Vector2f& src) const {
    return _w2g*src;
  }

  inline Eigen::Vector2f grid2world(const Eigen::Vector2f& src) const {
    return _g2w*src;
  }

};
