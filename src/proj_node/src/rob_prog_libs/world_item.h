#pragma once
#include "grid_map.h"
#include <Eigen/Geometry>
#include <set>

class LaserScanner;
class LaserScan;
using Eigen::Isometry2f;
class WorldItem;
using WorldItemSet=std::set<WorldItem*>;

class WorldItem {
public:

  static constexpr int CHILDREN_MAX_NUM = 10;
  Isometry2f pose_in_parent;
  WorldItem* parent;
  const GridMap* grid_map = 0;
  float radius = 1;
  WorldItemSet children;

protected:
  WorldItem(const GridMap* g,
            WorldItem* p,
            const Isometry2f& iso=Isometry2f::Identity());
  
public:
  WorldItem(const GridMap& g):
    WorldItem(&g, 0)
  {}

  WorldItem(WorldItem& p, const Isometry2f& iso=Isometry2f::Identity()):
    WorldItem(p.grid_map, &p, iso)
  {}

  ~WorldItem();
  
  Isometry2f globalPose() const;

  const GridMap& gridMap() const;
  
  bool isAncestor(const WorldItem& other) const;

  bool checkCollision(const WorldItem& other) const;
  
  bool checkCollision() const;

  inline bool move(const Isometry2f& iso) {
    Isometry2f restored_pose_in_parent=pose_in_parent;
    pose_in_parent = pose_in_parent * iso;
    if (checkCollision()) {
      pose_in_parent=restored_pose_in_parent;
      return false;
    }
    return true;
  }
            
  virtual void draw(Canvas& canvas, bool show_parent=false) const;

  virtual void tick(float time_interval);
};

class World: public WorldItem {
public:
  World(const GridMap& gmap);
  void draw(Canvas& canvas, bool show_parent=false) const;
};


class UnicyclePlatform: public WorldItem {
public:
  UnicyclePlatform(World& w, const Isometry2f& iso);
  float tv=0;
  float rv=0;
  void tick(float time_interval) override ;
  LaserScanner* laser_scanner = nullptr;
  LaserScan* laser_scan = nullptr;
};
