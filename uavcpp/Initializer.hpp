#include <iostream>
#include <set>

#include "RTree.h"
#include "geoutil.hpp"

typedef int ValueType;
typedef RTree<ValueType, double, 2, float> MyTree;
struct Rect {
  Rect() {}
  Rect(Polygon_2 poly) {
    double a_minX = std::numeric_limits<double>::max();
    double a_minY = std::numeric_limits<double>::max();
    double a_maxX = std::numeric_limits<double>::min();
    double a_maxY = std::numeric_limits<double>::min();
    for (VertexIterator V = poly.vertices_begin(); V != poly.vertices_end();
         ++V) {
      double x = CGAL::to_double(V->x());
      double y = CGAL::to_double(V->y());
      a_minX = std::min(a_minX, x);
      a_maxX = std::max(a_maxX, x);
      a_minY = std::min(a_minY, y);
      a_maxY = std::max(a_maxY, y);
    }
    min[0] = a_minX;
    min[1] = a_minY;

    max[0] = a_maxX;
    max[1] = a_maxY;
  }
  Rect(double a_minX, double a_minY, double a_maxX, double a_maxY) {
    min[0] = a_minX;
    min[1] = a_minY;

    max[0] = a_maxX;
    max[1] = a_maxY;
  }
  Rect Addbuffer(double buffer) {
    double a_minX = this->min[0] - buffer, a_minY = this->min[1] - buffer;
    double a_maxX = this->max[0] + buffer, a_maxY = this->max[1] + buffer;
    return Rect(a_minX, a_minY, a_maxX, a_maxY);
  }
  double min[2];
  double max[2];
};
struct Clusters {
  Clusters() { nrects = 0; }
  Clusters(vector<Polygon_2> in) {
    nrects = 0;
    for (Polygon_2 poly : in) {
      rects.push_back(Rect(poly));
      nrects++;
    }
  }
  void init() {
    for (int i = 0; i <= nrects; i++) {
      fa.push_back(i);
    }
    for (int i = 0; i < nrects; i++) {
      tree.Insert(rects[i].min, rects[i].max, i);
    }
  }
  vector<Polygon_2> GetClusterResult(const vector<Polygon_2> poly,
                                     vector<set<int>> *s, double buffer) {
    vector<Polygon_2> res;
    s->clear();
    //  get dsu graph
    for (int i = 0; i < nrects; i++) {
      Rect search_rect = rects[i].Addbuffer(buffer);

      auto MySearchCallback = [=](ValueType id) {
        // cout << "Hit data rect " << id << "\n";
        merge(id + 1, i + 1);
        return true;  // keep going
      };
      int x = tree.Search(search_rect.min, search_rect.max, MySearchCallback);
    }

    // clustering
    map<int, int> grp;
    int n_groups = 0;
    for (int i = 1; i <= nrects; i++) {
      if (fa[i] == i) grp[i] = n_groups++;
    }
    clusterset = vector<set<int>>(n_groups);

    for (int i = 1; i <= nrects; i++) {
      int p = grp[Get(i)];
      clusterset[p].insert(i - 1);
    }
    for (auto cls : clusterset) {
      s->push_back(cls);
    }

    // union polygons that are in the same cluster
    for (auto cls : clusterset) {
      vector<Point_2> points;
      for (auto id : cls) {
        for (VertexIterator V = poly[id].vertices_begin();
             V != poly[id].vertices_end(); ++V) {
          points.push_back(Point_2(V->x(), V->y()));
        }
      }
      Polygon_2 convexhull;
      CGAL::convex_hull_2(points.begin(), points.end(),
                          std::back_inserter(convexhull));
      res.push_back(convexhull);
    }
    return res;
  }

 private:
  int nrects;
  vector<Rect> rects;
  /*
   *  Rtree
   */
  MyTree tree;

  /*
   *  clustering by dsu
   */
  vector<int> fa;
  vector<set<int>> clusterset;
  int Get(int x) {
    if (x == fa[x]) return x;
    return fa[x] = Get(fa[x]);
  }
  void merge(int x, int y) {
    int p = Get(x), q = Get(y);
    // cout << p << " " << q << endl;
    if (p != q) fa[p] = q;
  }
};
