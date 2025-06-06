#include "laser_scanner.h"

LaserScanner::LaserScanner(LaserScan& scn,
                           WorldItem& par,
                           const Isometry2f& pos,
                           float f):
    WorldItem(par, pos),
    scan(scn),
    frequency(f),
    period(1./f)
{}

void LaserScanner::draw(Canvas& canvas, bool show_parent) const {
  scan.draw(canvas, *grid_map, globalPose());
}

void LaserScanner::getScan() {
  Isometry2f gp=globalPose();
  Isometry2f rotation=gp;
  rotation.translation().setZero();
  float angle_increment = (scan.angle_max-scan.angle_min)/scan.ranges.size();
    
  for (int i=0; i<scan.ranges.size(); ++i) {
    float beam_angle=scan.angle_min+angle_increment*i;
    Vector2f d(cos(beam_angle),
               sin(beam_angle));
    d=rotation * d;
    scan.ranges[i] = grid_map->scanRay(gp.translation(), d, scan.range_max);
  }
    
    
}

void LaserScanner::tick(float dt){
  elapsed_time+=dt;
  new_scan=false;
  if(elapsed_time>period)
    return;

  getScan();
  elapsed_time=0;
  new_scan=true;
}
