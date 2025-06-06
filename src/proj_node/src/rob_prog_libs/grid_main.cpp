#include "grid.h"
using namespace std;

using FloatGrid = Grid_<float>;

using StringGrid = Grid_<std::string>;

int main() {
  FloatGrid grid;
  grid.resize(10, 10);

  for (int r=0; r<grid.rows; ++r){
    for (int c=0; c< grid.cols; ++c) {
      grid(r,c)=(r+1)*0.1 * (c+1)*0.5;
    }
  }


  for (float r=0; r<grid.rows-1; r+=0.1){
    for (float c=0; c< grid.cols-1; c+=0.1) {
      cout << grid(Vector2f(c,r)) << " ";
    }
    cout << endl;
  }

  //cout << grid;

  
  // CT transf;

  // transf.reset(Eigen::Vector2f(10,20), 0.1);

  // Eigen::Vector2f grid_origin;
  // grid_origin.setZero();

  // cerr << transf.grid2world(grid_origin).transpose() << endl;

  // cerr << transf.grid2world(Eigen::Vector2f(1,1)).transpose() << endl;

 
}
