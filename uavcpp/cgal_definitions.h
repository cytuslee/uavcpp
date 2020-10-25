#ifndef POLYGON_COVERAGE_GEOMETRY_CGAL_DEFINITIONS_H_
#define POLYGON_COVERAGE_GEOMETRY_CGAL_DEFINITIONS_H_

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Partition_is_valid_traits_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/Point_generators_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/partition_2.h>
#include <CGAL/polygon_function_objects.h>
#include <CGAL/random_polygon_2.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef CGAL::Partition_traits_2<K> Traits;
typedef Traits::FT FT;
typedef Traits::Point_2 Point_2;
typedef Traits::Point_3 Point_3;
typedef Traits::Vector_2 Vector_2;
typedef Traits::Direction_2 Direction_2;
typedef Traits::Line_2 Line_2;
typedef Traits::Intersect_2 Intersect_2;
typedef Traits::Plane_3 Plane_3;
typedef Traits::Segment_2 Segment_2;
typedef Traits::Triangle_2 Triangle_2;
typedef Traits::Polygon_2 Polygon_2;
typedef Polygon_2::Vertex_const_iterator VertexConstIterator;
typedef Polygon_2::Vertex_const_circulator VertexConstCirculator;
typedef Polygon_2::Vertex_iterator VertexIterator;
typedef Polygon_2::Vertex_circulator VertexCirculator;
typedef Polygon_2::Edge_const_iterator EdgeConstIterator;
typedef Polygon_2::Edge_const_circulator EdgeConstCirculator;
typedef CGAL::Polygon_with_holes_2<K> PolygonWithHoles;
typedef CGAL::Exact_predicates_inexact_constructions_kernel InexactKernel;

#endif  // POLYGON_COVERAGE_GEOMETRY_CGAL_DEFINITIONS_H_
