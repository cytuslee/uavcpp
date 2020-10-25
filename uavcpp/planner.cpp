#include "planner.h"
bool CoveragePathPlan(InputNode &in) {
  TaskNode res;
  double footprint_width = in.footprint_width;
  double horizontalOverwrap = in.horizontalOverwrap;
  Point_2 start = in.start;
  Point_2 end = in.end;
  Polygon_2 polygon = in.polygon;
  vector<PolygonNode> polygons;
  PolygonNode p(in);
  TaskNode res;
  if (ComputeAllSweep(polygon, footprint_width, horizontalOverwrap,
                      &(p.cluster_waypoints), p.projectDir))) {
    polygons.push_back(p);
  }
  else {
    vector<polygon_2> decomposePolygons = decomposePolygon(in);
    for (auto poly : decomposePolygons) {
      polygonNode tmp(poly);
      ComputeAllSweep(poly, footprint_width, horizontalOverwrap,
                      &(tmp.cluster_waypoints), tmp.projectDir);
      polygons.push_back(tmp);
    }
  }
  for (auto Node : PolygonNode) {
    for (auto waypoints : Node.cluster_waypoints) {
      for (auto point : waypoints) {
        cout << point.x() << " , " << point.y() << endl;
      }
      cout << endl;
    }
  }
  return true;
}
