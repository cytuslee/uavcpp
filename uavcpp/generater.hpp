#ifndef GENERATER_H
#define GENERATER_H
#include <array>
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <string>
#include <unordered_map>

#include "geoutil.hpp"

const double PI = 3.1415926535897932384626;
bool computeSweep(const Polygon_2& in, const FT offset, const Direction_2& dir,
                  bool counter_clockwise, std::vector<Point_2>* waypoints) {
  waypoints->clear();
  if (in.is_clockwise_oriented()) return false;
  Line_2 sweep(Point_2(0.0, 0.0), dir);
  std::vector<Point_2> sorted_pts = sortVerticesToLine(in, sweep);
  sweep = Line_2(sorted_pts.front(), dir);
  Vector_2 offset_vector = sweep.perpendicular(sorted_pts.front()).to_vector();
  offset_vector = offset * offset_vector /
                  std::sqrt(CGAL::to_double(offset_vector.squared_length()));
  const CGAL::Aff_transformation_2<K> kOffset(CGAL::TRANSLATION, offset_vector);
  Segment_2 sweep_segment;
  bool intersect = false;
  bool has_sweep_segment =
      findSweepSegment(in, sweep, &sweep_segment, intersect);
  if (intersect == true) {
    waypoints->clear();
    return false;
  }
  while (has_sweep_segment == true) {
    // cout << "inter: " << intersect << endl;
    if (intersect == true) {
      waypoints->clear();
      return false;
    }
    // Align sweep segment.
    if (counter_clockwise) sweep_segment = sweep_segment.opposite();
    // Connect previous sweep.
    waypoints->push_back(sweep_segment.source());
    if (!sweep_segment.is_degenerate())
      waypoints->push_back(sweep_segment.target());
    // Offset sweep.
    sweep = sweep.transform(kOffset);
    has_sweep_segment = findSweepSegment(in, sweep, &sweep_segment, intersect);
    Segment_2 prev_sweep_segment =
        counter_clockwise ? sweep_segment.opposite() : sweep_segment;
    if (!has_sweep_segment &&
        !((!waypoints->empty() &&
           *std::prev(waypoints->end(), 1) == sorted_pts.back()) ||
          (waypoints->size() > 1 &&
           *std::prev(waypoints->end(), 2) == sorted_pts.back()))) {
      sweep = Line_2(sorted_pts.back(), dir);
      has_sweep_segment =
          findSweepSegment(in, sweep, &sweep_segment, intersect);
      // Do not add super close sweep.
      if (CGAL::squared_distance(sweep_segment, prev_sweep_segment) < 0.1)
        break;
    }
    // Swap directions.
    counter_clockwise = !counter_clockwise;
  }
  return true;
}
bool computeConvexSweep(Polygon_2& in, FT offset, vector<Point_2>* waypoints) {
  Polygon_2 convexhull;
  CGAL::convex_hull_2(in.vertices_begin(), in.vertices_end(),
                      std::back_inserter(convexhull));
  simplifyPolygon(convexhull);
  Direction_2 bestdir = getBestEdgeDirection(convexhull);

  bool counter_clockwise = true;
  return computeSweep(in, offset, bestdir, counter_clockwise, waypoints);
}
/*
calculate total length of waypoints
*/
double CalculatePathlength(const vector<Point_2> waypoints) {
  int sz = waypoints.size();
  if (sz < 2)
    return 0;
  else {
    double pathlength = 0;
    for (int i = 0; i < sz - 1; i++) {
      pathlength += distance(waypoints[i], waypoints[i + 1]);
    }
    return pathlength;
  }
}
/*
find SecondOptimaldir in all sweepdir
*/
bool computeSecondOptimalSweep(const Polygon_2& in, const FT offset,
                               Direction_2* SecondOptimaldir) {
  Polygon_2 convexhull;
  CGAL::convex_hull_2(in.vertices_begin(), in.vertices_end(),
                      std::back_inserter(convexhull));
  simplifyPolygon(convexhull);
  bool counter_clockwise = true;
  Direction_2 dir;
  bool hasAcceptableSweep = false;
  double min_distance = std::numeric_limits<double>::max();
  for (EdgeConstIterator edge = convexhull.edges_begin();
       edge != convexhull.edges_end(); ++edge) {
    double Vdistance = std::numeric_limits<double>::min();
    // if not has intersection
    vector<Point_2> waypoints;
    if (computeSweep(in, offset, edge->direction(), counter_clockwise,
                     &waypoints) == true) {
      for (VertexIterator V = convexhull.vertices_begin();
           V != convexhull.vertices_end(); V++) {
        Line_2 edgeLine(edge->source(), edge->target());
        Vdistance = max(Vdistance,
                        CGAL::to_double(CGAL::squared_distance(*V, edgeLine)));
      }
      hasAcceptableSweep = true;
      if (min_distance > Vdistance + eps) {
        min_distance = Vdistance;
        dir = edge->direction();
      }
    }
    //    cout << "hello" << endl;
  }
  if (hasAcceptableSweep == false) return false;
  *SecondOptimaldir = dir;
  return true;
}
/*
try to compute a bestsweep ,otherwise second optimal sweeps;
*/
bool ComputeAllSweep(const Polygon_2& in, double footprint_width,
                     double horizontalOverwrap,
                     vector<vector<Point_2> >* cluster_sweeps,
                     Direction_2& Dir) {
  FT offset = footprint_width * (1 - horizontalOverwrap);
  bool counter_clockwise = true;
  vector<Point_2> sweep;
  // compute bestdir
  Polygon_2 convexhull;
  CGAL::convex_hull_2(in.vertices_begin(), in.vertices_end(),
                      std::back_inserter(convexhull));
  simplifyPolygon(convexhull);
  Direction_2 bestdir = getBestEdgeDirection(convexhull);
  if (computeSweep(in, offset, bestdir, counter_clockwise, &sweep) == false) {
    // can not generate a no-insert zigzag path
    if (computeSecondOptimalSweep(in, offset, &bestdir) == false) {
      cluster_sweeps->clear();
      return false;
    }
  }
  // compute cluster_waypoints
  computeSweep(in, offset, bestdir, counter_clockwise, &sweep);
  cluster_sweeps->push_back(sweep);  // counter_clockwise

  std::reverse(sweep.begin(), sweep.end());
  cluster_sweeps->push_back(sweep);  // reverse
  computeSweep(in, offset, bestdir, !counter_clockwise, &sweep);
  cluster_sweeps->push_back(sweep);  // clockwise
  std::reverse(sweep.begin(), sweep.end());
  cluster_sweeps->push_back(sweep);  // reverse
  Dir = bestdir;
  return true;
}

#endif
