#ifndef MAPUTIL_hpp
#define MAPUTIL_hpp
#include <vector>

#include "cgal_definitions.h"
#include "geoutil.hpp"
using namespace std;
struct ProjectionDataset {
  double sizeX, sizeY;
  double trans[6];  //六参数
  double Nodata;
  const char* pszProj;                 //投影信息
  vector<vector<double> > RasterData;  //数据集
  double step;
  pair<int, int> getXYindex(double Xp, double Yp) {
    double dTemp = trans[1] * trans[5] - trans[2] * trans[4];
    int Xpixel =
        (trans[5] * (Xp - trans[0]) - trans[2] * (Yp - trans[3])) / dTemp + 0.5;
    int Yline =
        (trans[1] * (Yp - trans[3]) - trans[4] * (Xp - trans[0])) / dTemp + 0.5;
    return make_pair(Xpixel, Yline);
  }
  double Img2CoordX(double x, double y) {
    return trans[0] + trans[1] * x + trans[2] * y;
  }
  double Img2CoordY(double x, double y) {
    return trans[3] + trans[4] * x + trans[5] * y;
  }
  double GetSencingHight(Polygon_2 terrain, double hight) {
    double a_minX = std::numeric_limits<double>::max();
    double a_minY = std::numeric_limits<double>::max();
    double a_maxX = std::numeric_limits<double>::min();
    double a_maxY = std::numeric_limits<double>::min();
    for (VertexIterator V = terrain.vertices_begin();
         V != terrain.vertices_end(); ++V) {
      double x = CGAL::to_double(V->x());
      double y = CGAL::to_double(V->y());
      a_minX = std::min(a_minX, x);
      a_maxX = std::max(a_maxX, x);
      a_minY = std::min(a_minY, y);
      a_maxY = std::max(a_maxY, y);
    }
    pair<int, int> lp = getXYindex(a_minX, a_minY);
    pair<int, int> rp = getXYindex(a_maxX, a_maxY);
    int p_min[2], p_max[2];
    p_min[0] = min(lp.first, rp.first), p_max[0] = max(lp.first, rp.first);
    p_min[1] = min(lp.second, rp.second), p_max[1] = max(lp.second, rp.second);
    /*
   debug:  temporarily ignore terrains outof range,delete it when release
   */
    if (p_min[0] < 0 || p_min[1] < 0 || p_max[0] > sizeX || p_max[1] > sizeY)
      return -1;
    //-------------------------------------------------------------------
    double low = 10000, high = -10000;
    double sum = 0;
    int num = 0;
    for (int i = p_min[0]; i <= p_max[0]; i++) {
      for (int j = p_min[1]; j <= p_max[1]; j++) {
        double x = Img2CoordX(i, j);
        double y = Img2CoordY(i, j);
        if (pointInPolygon(terrain, Point_2(x, y)) == true) {
          if (RasterData[i][j] - Nodata > 1e-10) {
            low = min(low, RasterData[i][j]);
            high = max(high, RasterData[i][j]);
            num++;
            sum += RasterData[i][j];
          }
        }
      }
    }
    /*
     debug:  temporarily ignore terrains outof range,delete it when release
     */
    if (high == -10000 || low == 10000) return -1;
    //-------------------------------------------------------------------
    double avg = sum / num;
    // cout << high << " " << low << endl;
    // cout << avg << endl;
    return max(high + 50, avg + hight);
  }
  double GetTransferHight(double a_minX, double a_minY, double a_maxX,
                          double a_maxY) {
    pair<int, int> lp = getXYindex(a_minX, a_minY);
    pair<int, int> rp = getXYindex(a_maxX, a_maxY);
    int p_min[2], p_max[2];
    p_min[0] = min(lp.first, rp.first), p_max[0] = max(lp.first, rp.first);
    p_min[1] = min(lp.second, rp.second), p_max[1] = max(lp.second, rp.second);
    /*
debug:  temporarily ignore terrains outof range,delete it when release
*/
    if (p_min[0] < 0 || p_min[1] < 0 || p_max[0] > sizeX || p_max[1] > sizeY)
      return -1;
    //-------------------------------------------------------------------
    double high = -10000;
    for (int i = p_min[0]; i <= p_max[0]; i++) {
      for (int j = p_min[1]; j <= p_max[1]; j++) {
        if (RasterData[i][j] - Nodata > 1e-10)
          high = max(high, RasterData[i][j]);
      }
    }
    /*
     debug:  temporarily ignore terrains outof range,delete it when release
     */
    if (high == -10000) return -1;
    //-------------------------------------------------------------------
    return high + 50;
  }
};
#endif