#include <iostream>

#include "IOutils.hpp"
#include "TSP.hpp"
#include "TransformUtils.hpp"
#include "dataLoader.hpp"
#include "generater.hpp"
#include "planner.hpp"
using namespace std;
class UavPathPlanner {
 public:
  static string Plan(string inputjson) {
    // Load Profile
    prof p = LoadProfile("/Users/zongru/Desktop/uavcpp/profile.json");

    // Load terrains
    TERRAINS T = Readfromjsonstr(inputjson.c_str());
    vector<Polygon_2> pgns = T.terrains;
    // Clustering
    double buffer = 30;
    Clusters cls(pgns);
    cls.init();
    vector<set<int>> s;
    vector<Polygon_2> res = cls.GetClusterResult(pgns, &s, buffer);
    // Load DEM
    ProjectionDataset Maplow = LoadFromDEM(p.DEMlow.c_str());
    ProjectionDataset Maphigh = LoadFromDEM(p.DEMhigh.c_str());
    // generate Task for terrains
    vector<InputNode> Task2D;
    for (auto poly : res) {
      /*
     debug:  temporarily ignore terrains outof range,delete it when release
     */
      double h = Maphigh.GetSencingHight(poly, p.hight);
      if (h != -1) {
        //-----------------------------------------------------------------
        Task2D.push_back(InputNode(poly, p.getfootprintwidth(),
                                   p.horizontalOverwrap, *poly.vertices_begin(),
                                   *poly.vertices_begin()));
      }
    }
    vector<uavtask> TerrainTask;

    for (auto t : Task2D) {
      TaskNode task = CoveragePathPlan(t);
      double h = Maphigh.GetSencingHight(t.polygon, p.hight);
      uavtask uavtsk;
      for (auto node : task.TaskSequence) {
        for (Point_2 p : node.waypoints) {
          uavtsk.WayPoints.push_back(
              way_point{CGAL::to_double(p.x()), CGAL::to_double(p.y()), h});
        }
        uavtsk.IsTerrain = true;
        uavtsk.hight = h;
      }
      uavtsk.WayPoints.push_back(way_point{CGAL::to_double(task.end.x()),
                                           CGAL::to_double(task.end.y()), h});
      TerrainTask.push_back(uavtsk);
    }
    // calculate sequence by TSP opt-3 algorithm
    vector<Point_2> posvector;
    Point_2 start = p.StartPos;
    double startx = CGAL::to_double(start.x()),
           starty = CGAL::to_double(start.y());
    LatLonToUTMXY(startx, starty);
    start = Point_2(startx, starty);
    posvector.push_back(start);
    for (auto t : Task2D) {
      posvector.push_back(t.start);
    }
    double transformhight = p.transformhight;
    way_point prep{startx, starty, p.transformhight},
        endp{startx, starty, p.transformhight};
    vector<int> TransSquence = TSPsequence(posvector);
    int sz = TransSquence.size();
    // generate Output
    vector<uavtask> OutputTask;
    for (int i = 0; i < sz - 1; i++) {
      if (i == 0) {
        continue;
      } else {
        int id = TransSquence[i] - 1;
        // trans
        uavtask transtmp;
        transtmp.IsTerrain = false;
        transtmp.WayPoints.push_back(prep);
        way_point p = TerrainTask[id].WayPoints.at(0);
        transtmp.WayPoints.push_back(p);
        transtmp.hight =
            max(p.h, Maphigh.GetTransferHight(p.x, p.y, prep.x, prep.y));
        // cout << transtmp.hight << endl;
        transtmp.Reconstruct();
        OutputTask.push_back(transtmp);
        // terrains
        TerrainTask[id].Reconstruct();
        OutputTask.push_back(TerrainTask[id]);
        prep = p;
        if (i == sz - 2) {
          // trans
          uavtask transendtmp;
          transendtmp.IsTerrain = false;
          transendtmp.WayPoints.push_back(prep);
          transendtmp.WayPoints.push_back(endp);
          transendtmp.hight = max(
              prep.h, Maphigh.GetTransferHight(prep.x, prep.y, endp.x, endp.y));
          transendtmp.Reconstruct();
          OutputTask.push_back(transendtmp);
        }
      }
    }
    // Transform to LatLon
    // generate jsonOutPut
    int zone = floor(CGAL::to_double(p.StartPos.y())) / 6 + 31;
    nlohmann::json j;
    double shootPhotoDistanceInterval = p.GetshootPhotoDistanceInterval();
    j["startLat"] = CGAL::to_double(p.StartPos.x());
    j["startLont"] = CGAL::to_double(p.StartPos.y());
    for (uavtask tsk : OutputTask) {
      nlohmann::json tmp;
      tmp["takephoto"] = tsk.IsTerrain;
      tmp["shootPhotoDistanceInterval"] = shootPhotoDistanceInterval;
      for (way_point p : tsk.WayPoints) {
        TransformToLatLon(p.x, p.y, zone, false);
        // cout << fixed << setprecision(5) << p.x << " ," << p.y << endl;
        vector<double> c_vector{p.x, p.y, p.h};
        nlohmann::json p_vec(c_vector);
        tmp["coordinates"].push_back(p_vec);
      }
      j["tasks"].push_back(tmp);
    }
    string str = j.dump();
    return str;
  }
};
/*test entry for the algorithm*/
int main() {
  FILE *stream;
  char *contents;
  int fileSize = 0;
  stream = fopen("/Users/zongru/Desktop/uavcpp/sampledata.txt", "rb");
  fseek(stream, 0L, SEEK_END);
  fileSize = ftell(stream);
  fseek(stream, 0L, SEEK_SET);
  contents = (char *)malloc(fileSize + 1);
  size_t size = fread(contents, 1, fileSize, stream);
  contents[size] = 0;
  fclose(stream);
  string content = (string)contents;
  // cout << content << endl;
  string outputjson = UavPathPlanner::Plan(content);
  cout << outputjson << endl;
  return 0;
}