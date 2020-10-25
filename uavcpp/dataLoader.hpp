#ifndef DATALOADER_hpp
#define DATALOADER_hpp
#include <gdal_priv.h>
#include <ogr_feature.h>
#include <ogr_geometry.h>
#include <ogr_srs_api.h>
#include <ogrsf_frmts.h>

#include <cstring>
#include <fstream>
#include <iostream>

#include "IOutils.hpp"
#include "Initializer.hpp"
#include "TransformUtils.hpp"
#include "json.hpp"
/*
load aoidata from geojson,and Mapdata from tiffs;
TODO : use gdal to transform coordinates
*/
using namespace std;
struct prof {
  string DEMlow;
  string DEMhigh;
  double hight;               //航高(m)
  Point_2 StartPos;           //起飞点
  double horizontalOverwrap;  //重叠率
  double verticalOverwrap;
  double f;                //焦距 (mm)
  double pixelsize;        //像元尺寸(mm)
  int ImgsizeX, ImgsizeY;  //像幅范围
  double transformhight;   // default : 返航高度
  double getfootprintwidth() { return ImgsizeX * pixelsize * hight / f; }
  double getfootprintlength() { return ImgsizeY * pixelsize * hight / f; }
  double GetshootPhotoDistanceInterval() {
    return (1 - verticalOverwrap) * ImgsizeY * pixelsize * hight / f;
  }
};
prof LoadProfile(string filename) {
  using namespace nlohmann;
  prof p;
  std::ifstream ifs(filename);
  nlohmann::json j;
  ifs >> j;
  if (j == NULL) {
    cout << "null" << endl;
    return p;
  }
  p.DEMhigh = j["DEMhigh"].get<std::string>();
  p.DEMlow = j["DEMlow"].get<std::string>();
  p.hight = j["hight"].get<double>();
  double x = j["StartLat"].get<double>(), y = j["StartLont"].get<double>();
  p.StartPos = Point_2(x, y);
  p.horizontalOverwrap = j["horizontalOverwrap"].get<double>();
  p.verticalOverwrap = j["verticalOverwrap"].get<double>();
  p.pixelsize = j["pixelsize"].get<double>();
  p.f = j["f"].get<double>();
  p.ImgsizeX = j["ImgsizeX"].get<int>();
  p.ImgsizeY = j["ImgsizeY"].get<int>();
  p.transformhight = j["transformhight"].get<double>();
  return p;
}

struct TERRAINS {
  vector<Point_2> pos;
  vector<Polygon_2> terrains;
};
Polygon_2 OGRtoPolygon(OGRPolygon *OGRpoly) {
  Polygon_2 res;
  OGRLinearRing *boundary = OGRpoly->getExteriorRing();
  int pointcount = boundary->getNumPoints();
  for (int i = 0; i < pointcount; i++) {
    double x = boundary->getX(i), y = boundary->getY(i);
    LatLonToUTMXY(y, x);
    res.push_back(Point_2(y, x));
  }
  return res;
}

TERRAINS Readfromjsonstr(const char *content) {
  TERRAINS T;
  GDALAllRegister();
  GDALDataset *poDS;
  poDS = (GDALDataset *)GDALOpenEx(content, GDAL_OF_VECTOR, NULL, NULL, NULL);
  if (poDS == NULL) {
    printf("Open failed.\n");
    exit(1);
  }
  OGRLayer *poLayer;
  poLayer = poDS->GetLayerByName("OGRGeoJson");
  int i = 0;
  for (auto &poFeature : poLayer) {
    //获取要素类定义
    OGRFeatureDefn *poFDefn = poLayer->GetLayerDefn();
    int iField;
    double x, y;
    for (iField = 0; iField < poFDefn->GetFieldCount(); iField++) {
      OGRFieldDefn *poFieldDefn = poFDefn->GetFieldDefn(iField);
      if ((string)poFieldDefn->GetNameRef() == "latitude") {
        x = poFeature->GetFieldAsDouble(iField);
      }
      if ((string)poFieldDefn->GetNameRef() == "longitude")
        y = poFeature->GetFieldAsDouble(iField);
    }
    if (y < 112.52 || y > 112.57 || x < 23.152 || x > 23.192) {
      // cout << x << " " << y << endl;
      continue;
    }
    LatLonToUTMXY(y, x);
    T.pos.push_back(Point_2(x, y));
    //获取几何形状
    OGRGeometry *poGeometry;
    poGeometry = poFeature->GetGeometryRef();
    if (poGeometry != NULL &&
        wkbFlatten(poGeometry->getGeometryType()) == wkbPolygon) {
      OGRPolygon *poly = (OGRPolygon *)poGeometry;
      T.terrains.push_back(OGRtoPolygon(poly));
    }
  }
  return T;
}
ProjectionDataset LoadFromDEM(const char *filename) {
  ProjectionDataset Map;
  readGeoRaster(filename, Map);
  return Map;
}
#endif