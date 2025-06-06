#pragma once
#include "laser_scan.h"
#include "world_item.h"

class LaserScanner: public WorldItem{
public:
  LaserScan& scan;

  LaserScanner(LaserScan& scn,
               WorldItem& par,
               const Isometry2f& pos,
               float frequency=10.f);

  void draw(Canvas& canvas, bool show_parent) const;

  void tick(float time_interval) override ;
  
  inline bool newScan() const {return new_scan;}
  void getScan();

protected:
  float frequency;
  float period;
  float elapsed_time=0;
  bool new_scan=false;
};
