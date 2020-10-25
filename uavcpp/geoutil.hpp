#ifndef GEOUTILS_H
#define GEOUTILS_H

// c++ libs
#include <algorithm>
#include <cmath>
#include <iostream>
#include <list>
#include <stack>
#include <vector>
using namespace std;
#include "cgal_definitions.h"
const double eps = 1e-6;
double distance(Point_2 a, Point_2 b) {
  return std::sqrt(CGAL::to_double(CGAL::squared_distance(a, b)));
}
bool pointInPolygon(const Polygon_2& polygon, const Point_2& p) {
  // Point inside outer boundary.
  CGAL::Bounded_side result = CGAL::bounded_side_2(
      polygon.vertices_begin(), polygon.vertices_end(), p, K());
  if (result == CGAL::ON_UNBOUNDED_SIDE) return false;
  return true;
}

void simplifyPolygon(Polygon_2& polygon) {
  std::vector<Polygon_2::Vertex_circulator> v_to_erase;

  Polygon_2::Vertex_circulator vc = polygon.vertices_circulator();
  // Find collinear vertices.
  do {
    if (CGAL::collinear(*std::prev(vc), *vc, *std::next(vc))) {
      v_to_erase.push_back(vc);
    }
  } while (++vc != polygon.vertices_circulator());

  // Remove intermediate vertices.
  for (std::vector<Polygon_2::Vertex_circulator>::reverse_iterator rit =
           v_to_erase.rbegin();
       rit != v_to_erase.rend(); ++rit) {
    polygon.erase(*rit);
  }
}
void sortVertices(Polygon_2 polygon) {
  if (polygon.is_clockwise_oriented()) polygon.reverse_orientation();
}
bool isAdjacent(Polygon_2 a, Polygon_2 b) {
  for (VertexIterator v = a.vertices_begin(); v != a.vertices_end(); v++) {
    for (VertexIterator p = a.vertices_begin(); p != a.vertices_end(); p++) {
      if (v == p) return true;
    }
  }
  return false;
}
Direction_2 getBestEdgeDirection(const Polygon_2& in) {
  // Get all directions.
  Direction_2 dir;
  double min_distance = std::numeric_limits<double>::max();
  for (EdgeConstIterator edge = in.edges_begin(); edge != in.edges_end();
       ++edge) {
    double Vdistance = std::numeric_limits<double>::min();
    for (VertexIterator V = in.vertices_begin(); V != in.vertices_end(); V++) {
      Line_2 edgeLine(edge->source(), edge->target());
      Vdistance =
          max(Vdistance, CGAL::to_double(CGAL::squared_distance(*V, edgeLine)));
    }
    // cout << Vdistance << endl;
    if (min_distance > Vdistance + eps) {
      min_distance = Vdistance;
      dir = edge->direction();
    }
  }
  return dir;
}

std::vector<Direction_2> getAllSweepableEdgeDirections(const Polygon_2& in) {
  // Get all directions.
  std::vector<Direction_2> dirs;
  for (EdgeConstIterator it = in.edges_begin(); it != in.edges_end(); ++it) {
    // Check if this edge direction is already in the set.
    std::vector<Direction_2>::iterator last =
        std::find_if(dirs.begin(), dirs.end(), [&it](const Direction_2& dir) {
          return CGAL::orientation(dir.vector(), it->to_vector()) ==
                 CGAL::COLLINEAR;
        });
    if (last != dirs.end()) continue;
    dirs.push_back(it->direction());
  }
  return dirs;
}
bool findIntersections(const Polygon_2& p, const Line_2& l,
                       std::vector<Point_2>& intersections) {
  typedef CGAL::cpp11::result_of<Intersect_2(Segment_2, Line_2)>::type
      Intersection;

  for (EdgeConstIterator it = p.edges_begin(); it != p.edges_end(); ++it) {
    Intersection result = CGAL::intersection(*it, l);
    if (result) {
      if (const Segment_2* s = boost::get<Segment_2>(&*result)) {
        intersections.push_back(s->source());
        intersections.push_back(s->target());
      } else {
        intersections.push_back(*boost::get<Point_2>(&*result));
      }
    }
  }

  // Sort.
  Line_2 perp_l = l.perpendicular(l.point(0));
  std::sort(intersections.begin(), intersections.end(),
            [&perp_l](const Point_2& a, const Point_2& b) -> bool {
              return CGAL::has_smaller_signed_distance_to_line(perp_l, a, b);
            });
  if (intersections.size() >= 3) {
    int sz = intersections.size();
    for (int i = 0; i < sz - 1; i++) {
      if (pointInPolygon(p, CGAL::midpoint(intersections[i],
                                           intersections[i + 1])) == false) {
        return false;
      }
    }
  }
  return true;
}
bool findSweepSegment(const Polygon_2& p, const Line_2& l,
                      Segment_2* sweep_segment, bool& intersect) {
  std::vector<Point_2> intersections;
  // cout << "hello" << endl;
  if (findIntersections(p, l, intersections) == false) {
    intersect = true;
    return true;
  }
  if (intersections.empty()) {
    return false;
  }
  *sweep_segment = Segment_2(intersections.front(), intersections.back());
  // cout << sweep_segment->source() << " " << sweep_segment->target() << endl;
  return true;
}
std::vector<Point_2> sortVerticesToLine(const Polygon_2& p, const Line_2& l) {
  // Copy points.
  std::vector<Point_2> pts(p.size());
  std::vector<Point_2>::iterator pts_it = pts.begin();
  for (VertexConstIterator it = p.vertices_begin(); it != p.vertices_end();
       ++it) {
    *(pts_it++) = *it;
  }

  // Sort.
  std::sort(pts.begin(), pts.end(),
            [&l](const Point_2& a, const Point_2& b) -> bool {
              return CGAL::has_smaller_signed_distance_to_line(l, a, b);
            });

  return pts;
}
std::vector<Polygon_2> decomposePolygon(Polygon_2 polygon) {
  std::list<Polygon_2> partialCGALPolygons;
  Traits partitionTraits;
  CGAL::optimal_convex_partition_2(
      polygon.vertices_begin(), polygon.vertices_end(),
      std::back_inserter(partialCGALPolygons), partitionTraits);
  std::vector<Polygon_2> decomposedPolygons;
  for (Polygon_2 partialCGALPolygon : partialCGALPolygons) {
    simplifyPolygon(partialCGALPolygon);
    sortVertices(partialCGALPolygon);
    decomposedPolygons.push_back(partialCGALPolygon);
  }

  return decomposedPolygons;
}

#endif
