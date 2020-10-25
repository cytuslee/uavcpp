#include <iomanip>

#include "Astarutils.cpp"
#include "IOutils.hpp"
//#include "Resample.cpp"
#include "TransformUtils.hpp"
int main() {
  ProjectionDataset Map;
  readGeoRaster("SAMPLE.tif", Map);
  cout << Map.sizeX << " " << Map.sizeY << endl;
  Map.step = 30;
  SquareGrid grid = SquareGrid(Map);

  pair<int, int> Start = Map.getXYindex(658531.466646246, 2562772.33902094);
  pair<int, int> End = Map.getXYindex(657804.008274284, 2563783.44929696);
  //   cout << fixed << setprecision(2)
  //        << Img2CoordX(Map.trans, Start.first, Start.second) << " "
  //        << Img2CoordY(Map.trans, Start.first, Start.second) << endl;
  //   cout << fixed << setprecision(2)
  //        << Img2CoordX(Map.trans, End.first, End.second) << " "
  //        << Img2CoordY(Map.trans, End.first, End.second) << endl;
  cout << Start.first << " " << Start.second << endl;
  cout << End.first << " " << End.second << endl;
  cout << Map.RasterData[Start.first][Start.second] << endl;
  cout << Map.RasterData[End.first][End.second] << endl;
  GridLocation start{Start.first, Start.second}, goal{End.first, End.second};
  std::unordered_map<GridLocation, GridLocation> came_from;
  std::unordered_map<GridLocation, double> cost_so_far;
  a_star_search(grid, start, goal, came_from, cost_so_far);
  std::vector<GridLocation> path = reconstruct_path(start, goal, came_from);
  for (auto p : path) {
    cout << p.x << "," << p.y << " " << Map.RasterData[p.x][p.y] << " "
         << cost_so_far[p] << endl;
  }
  // draw_grid(grid, nullptr, &came_from, nullptr, &start, &goal);
  // draw_grid(grid, nullptr, nullptr, &path, &start, &goal);
  // draw_grid(grid, &cost_so_far, nullptr, nullptr, &start, &goal);
}
// 23.1663,112.5487
// 23.1755,112.5417
// 658531.466646246,2562772.33902094
// 657804.008274284,2563783.44929696
