#ifndef PLANNER_H
#define PLANNER_H
#include "generater.hpp"
/*
statment for output
*/
struct way_point {
  double x, y, h;  // x: Lontitude y:Latitude
};
struct uavtask {
  vector<way_point> WayPoints;
  bool IsTerrain;
  // double heading;  // heading angle to north
  // double shootphototimeInterval;
};
/*
coverageplanning utils for a task polygon
*/
struct PolygonNode {
  PolygonNode() {}
  PolygonNode(Polygon_2 polygon) : polygon(polygon) {}
  Polygon_2 polygon;
  Direction_2 projectDir;                      //headdir
  vector<vector<Point_2> > cluster_waypoints;  // four acceptable waypoints
  vector<double> pathlength;
};
struct PartialTaskNode {
  PartialTaskNode() {}
  PartialTaskNode(Direction_2 projectDir, vector<Point_2> waypoints)
      : projectDir(projectDir), waypoints(waypoints) {}
  Direction_2 projectDir;     //headdir
  vector<Point_2> waypoints;  // task waypoints;
};
struct TaskNode {
  Point_2 end;
  vector<PartialTaskNode> TaskSequence;  // cover a polygon
  double pathlength;                     // pathlength
};
struct InputNode {
  InputNode() {}
  InputNode(Polygon_2 polygon, double footprint_width,
            double horizontalOverwrap, Point_2 start, Point_2 end) {
    this->polygon = polygon;
    this->footprint_width = footprint_width;
    this->horizontalOverwrap = horizontalOverwrap;
    this->start = start;
    this->end = end;
  }
  Polygon_2 polygon;
  double footprint_width;
  double horizontalOverwrap;
  Point_2 start;
  Point_2 end;
};
TaskNode CoveragePathPlan(InputNode &in) {
  double footprint_width = in.footprint_width;
  double horizontalOverwrap = in.horizontalOverwrap;
  Point_2 start = in.start;
  Point_2 end = in.end;
  Polygon_2 polygon = in.polygon;
  vector<PolygonNode> polygons;
  PolygonNode p(polygon);
  if (ComputeAllSweep(polygon, footprint_width, horizontalOverwrap,
                      &(p.cluster_waypoints), p.projectDir) == true) {
    polygons.push_back(p);
  } else {
    vector<Polygon_2> decomposePolygons = decomposePolygon(polygon);
    for (Polygon_2 poly : decomposePolygons) {
      PolygonNode tmp(poly);
      // for (VertexIterator itr = poly.vertices_begin();
      //      itr != poly.vertices_end(); itr++) {
      //   cout << itr->x() << ", " << itr->y() << endl;
      // }
      // cout << endl;
      ComputeAllSweep(poly, footprint_width, horizontalOverwrap,
                      &(tmp.cluster_waypoints), tmp.projectDir);
      polygons.push_back(tmp);
    }
  }
  for (auto &Node : polygons) {
    vector<double> lvector;
    for (auto waypoints : Node.cluster_waypoints) {
      Node.pathlength.push_back(CalculatePathlength(waypoints));
    }
  }
  TaskNode res;
  // calculate  optimal traverse path for polygons
  std::vector<int> permutation(polygons.size());
  std::iota(permutation.begin(), permutation.end(), 0);
  double minPathLength = std::numeric_limits<double>::max();
  int adjacencyCriteria = 1;
  int adjacencyCount = 0;
  do {
    // assigon the firt as a specific poluygon
    // if (permutation.front() != 0) {
    //   continue;
    // }
    // count adjecent polygon
    for (auto itr = permutation.begin(); itr != permutation.end() - 1; ++itr) {
      if (isAdjacent(polygons.at(*itr).polygon,
                     polygons.at(*(itr + 1)).polygon)) {
        ++adjacencyCount;
      }
    }
    if (adjacencyCriteria < permutation.size() &&
        adjacencyCount < adjacencyCriteria) {
      continue;
    }
    double candidatepathLength = 0;
    Point_2 pre = start;
    vector<PartialTaskNode> candidateTaskSequence;
    for (auto itr = permutation.begin(); itr != permutation.end(); ++itr) {
      PartialTaskNode partialNode;
      partialNode.projectDir = polygons.at(*itr).projectDir;
      double mintraversedistance = std::numeric_limits<double>::max();
      vector<double> candidatelength = polygons.at(*itr).pathlength;
      vector<vector<Point_2> > candidateWaypoints =
          polygons.at(*itr).cluster_waypoints;
      if (itr == permutation.end() - 1) {
        int sz = candidatelength.size();
        for (int i = 0; i < sz; i++) {
          if (distance(candidateWaypoints[i].back(), end) +
                  candidatelength.at(i) +
                  distance(candidateWaypoints[i].front(), pre) <
              mintraversedistance) {
            mintraversedistance = distance(candidateWaypoints[i].back(), end) +
                                  candidatelength.at(i) +
                                  distance(candidateWaypoints[i].front(), pre);
            partialNode.waypoints = candidateWaypoints[i];
          }
        }
      } else {
        int sz = candidatelength.size();
        for (int i = 0; i < sz; i++) {
          if (candidatelength.at(i) +
                  distance(candidateWaypoints[i].front(), pre) <
              mintraversedistance) {
            mintraversedistance = candidatelength.at(i) +
                                  distance(candidateWaypoints[i].front(), pre);
            partialNode.waypoints = candidateWaypoints[i];
          }
        }
      }
      candidatepathLength += mintraversedistance;
      pre = partialNode.waypoints.front();
      candidateTaskSequence.push_back(partialNode);
    }
    if (candidatepathLength >= 0 && candidatepathLength < minPathLength) {
      res.end = end;
      res.TaskSequence = candidateTaskSequence;
      res.pathlength = candidatepathLength;
      minPathLength = candidatepathLength;
    }
    // cout << candidatepathLength << endl;
  } while (next_permutation(permutation.begin(), permutation.end()));
  return res;
}

#endif
