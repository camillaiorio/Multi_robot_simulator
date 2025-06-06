#include "world_item.h"
#include <iostream>

using namespace std;
using Eigen::Rotation2Df;

WorldItem::~WorldItem() {
  if (parent)
    parent->children.erase(this);
}

WorldItem::WorldItem(const GridMap* g, WorldItem* p, const Isometry2f& iso) :
  grid_map(g),
  parent(p),
  pose_in_parent(iso){
  if (!p)
    return;
  p->children.insert(this);
}

bool WorldItem::isAncestor(const WorldItem& other) const {
  const WorldItem* a=this;
  while (a) {
    if (a==&other)
      return true;
    a=a->parent;
  }
  return false;
}

Isometry2f WorldItem::globalPose() const {
  if (!parent) return pose_in_parent;
  return parent->globalPose() * pose_in_parent;
}


const GridMap& WorldItem::gridMap() const {
  if (grid_map) return *grid_map;

  WorldItem* p = parent;
  if (p) {
    while (p->parent) {
      if (p->grid_map) return (*p->grid_map);
      p = p->parent;
    }
  }

  throw std::runtime_error("No GridMap available in this branch");
  return *grid_map;
}

void WorldItem::draw(Canvas& canvas, bool show_parent) const {
  Vector2f center = grid_map->world2grid(globalPose().translation());
  int radius_px = radius / grid_map->resolution();
  drawCircle(canvas, center, radius_px, 0);

  Vector2f x_in_item = {radius, 0};
  Vector2f x_in_world = globalPose() * x_in_item;
  Vector2f x_in_grid = grid_map->world2grid(x_in_world);
  drawLine(canvas, center, x_in_grid, 0);

  if (show_parent == true && parent != nullptr) {
    Vector2f parent_in_grid =
      grid_map->world2grid(parent->globalPose().translation());
    drawLine(canvas, center, parent_in_grid, 100);
  }
  for (auto child: children)
    child->draw(canvas, show_parent);

}

bool WorldItem:: checkCollision() const  {
  Isometry2f pose= globalPose();
  int radius_px=radius/ grid_map->resolution();
  int r2=radius_px*radius_px;
  
  Vector2f origin_px=grid_map->world2grid(pose.translation());
  int r0=origin_px.y();
  int c0=origin_px.x();
  for (int r=-radius_px; r<=radius_px; ++r) {
    for (int c=-radius_px; c<=radius_px; ++c){
      if (r*r+c*c>r2)
        continue;
      if (! grid_map->inside(r+r0, c+c0))
        return true;

      if ((*grid_map)(r+r0, c+c0)<127)
        return true;
    }
  }
  for(auto child: children)
    if (child->checkCollision())
      return true;
  
  if (parent && !(parent->parent)) {
    for (const auto* sibling: parent->children) {
      if (sibling==this)
        continue;
      if (checkCollision(*sibling))
        return true;
      if (sibling->checkCollision(*this))
        return true;
    }
  }
  return false;
}

bool WorldItem::checkCollision(const WorldItem& other) const {
  if (isAncestor(other))
    return false;
  if (other.isAncestor(*this))
    return false;
  // calculate the distance between me and other
  Isometry2f my_pose=globalPose();
  Isometry2f other_pose=other.globalPose();
  Vector2f delta=my_pose.translation()-other_pose.translation();
  float distance=delta.norm();
  if (distance<(radius+other.radius))
    return true;
  for(auto child: children)
    if (child->checkCollision(other))
      return true;

  return false;
  
}


void WorldItem::tick(float dt){
  for (auto child: children)
    child->tick(dt);
}

World::World(const GridMap& gmap):
  WorldItem(gmap){}

void World::draw(Canvas& canvas, bool show_parent) const {
  grid_map->draw(canvas);
  Vector2f origin=grid_map->world2grid(Vector2f::Zero());
  Vector2f x0=origin+Vector2f(0,grid_map->rows/2);
  Vector2f x1=origin-Vector2f(0,grid_map->rows/2);
  drawLine(canvas, x0, x1, 200); 
  Vector2f y0=origin+Vector2f(grid_map->cols/2,0);
  Vector2f y1=origin-Vector2f(grid_map->cols/2,0);
  drawLine(canvas, y0, y1, 200);

  for (auto child: children)
     child->draw(canvas, false);

}


UnicyclePlatform::UnicyclePlatform(World& w, const Isometry2f& iso):
  WorldItem(w, iso){}

void UnicyclePlatform::tick(float dt) {
  WorldItem::tick(dt);
  Isometry2f motion=Isometry2f::Identity();
  motion.translation() << tv*dt, 0;
  motion.linear()=Rotation2Df(rv*dt).matrix();
  if (! move(motion)){
    tv=0;
    rv=0;
  }
}


