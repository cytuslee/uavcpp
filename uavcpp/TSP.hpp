#ifndef TSP_hpp
#define TSP_hpp
#include <math.h>

#include <algorithm>
#include <ctime>
#include <iostream>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

#include "geoutil.hpp"
using namespace std;
typedef vector<vector<double>> matrix;
typedef vector<pair<double, double>> row;
typedef tuple<int, int, int> triple;

double inf = numeric_limits<double>::infinity();
vector<int> route;
matrix distances;

double calc_len() {
  double res = 0;
  for (int i = 0; i < route.size() - 1; i++)
    res += distances[route[i]][route[i + 1]];
  return res;
}

void print_tour() {
  for (int i : route) cout << i << ' ';
  cout << endl;
}

double dist(double x, double y, double x1, double y1) {
  return sqrt((x1 - x) * (x1 - x) + (y1 - y) * (y1 - y));
}

int proper_min(vector<int>& visited, int a, int num) {
  double mini = inf;
  int ind = 0;
  for (int i = 0; i < num; i++) {
    if (distances[a][i] < mini)
      if (visited[i] == 0) {
        ind = i;
        mini = distances[a][i];
      }
  }
  return ind;
}

void greedy(vector<int>& tour, int num) {
  vector<int> visited;
  for (int i = 0; i < num; i++) visited.push_back(0);
  int city = num - 1;
  for (int i = 0; i < num; i++) {
    visited[city] = 1;
    tour.push_back(city);
    city = proper_min(visited, city, num);
  }
}

double reverse_segment_if_better(vector<int>& tour, int i, int j, int k) {
  int A = tour[i - 1];
  int B = tour[i];
  int C = tour[j - 1];
  int D = tour[j];
  int E = tour[k - 1];
  int F = tour[k % tour.size()];

  double d0 = (distances[A][B] + distances[C][D] + distances[E][F]);
  double d1 = (distances[A][C] + distances[B][D] + distances[E][F]);
  double d2 = (distances[A][B] + distances[C][E] + distances[D][F]);
  double d3 = (distances[A][D] + distances[E][B] + distances[C][F]);
  double d4 = (distances[F][B] + distances[C][D] + distances[E][A]);

  if (d0 > d1) {
    reverse(tour.begin() + i, tour.begin() + j);
    return -d0 + d1;
  } else if (d0 > d2) {
    reverse(tour.begin() + j, tour.begin() + k);
    return -d0 + d2;
  } else if (d0 > d4) {
    reverse(tour.begin() + i, tour.begin() + k);
    return -d0 + d4;
  } else if (d0 > d3) {
    vector<int> tmp;
    for (int z = j; z < k; z++) tmp.push_back(route[z]);
    for (int z = i; z < j; z++) tmp.push_back(route[z]);
    for (int f = i; f < k; f++) route[f] = tmp[f];
    return -d0 + d3;
  }
  return 0;
}

vector<triple> combinations(int n) {
  vector<triple> res;
  for (int i = 1; i < n; i++)
    for (int j = i + 2; j < n; j++)
      for (int k = j + 2; k < n + 1; k++) res.emplace_back(make_tuple(i, j, k));
  return res;
}

void two_opt(vector<int>& tour, int num, clock_t time) {
  double d = calc_len();
  while ((float)(clock() - time) / CLOCKS_PER_SEC < 9.5) {
    for (int i = 1; i < num - 2; i++)
      for (int j = i + 2; j < num; j++) {
        int A = tour[i - 1];
        int B = tour[i];
        int C = tour[j - 1];
        int D = tour[j];
        double d0 = distances[A][B] + distances[C][D];
        double d1 = distances[A][C] + distances[B][D];
        if (d0 > d1) reverse(tour.begin() + i, tour.begin() + j);
      }
    double new1 = calc_len();
    if ((d - new1) == 0) break;
    d = new1;
  }
}

void three_opt(vector<int>& tour, clock_t time) {
  double delta = 0;
  while ((float)(clock() - time) / CLOCKS_PER_SEC < 9.8) {
    vector<triple> comb = combinations(tour.size());
    for (int i = 0; i < comb.size(); i++)
      delta += reverse_segment_if_better(tour, get<0>(comb[i]), get<1>(comb[i]),
                                         get<2>(comb[i]));
    if (delta >= 0) break;
  }
}
vector<int> TSPsequence(vector<Point_2> pointvector) {
  clock_t begin_time = clock();
  int num = pointvector.size();
  double x, y;
  row points;
  for (int i = 0; i < num; i++) {
    x = CGAL::to_double(pointvector.at(i).x());
    y = CGAL::to_double(pointvector.at(i).y());
    points.push_back(make_pair(x, y));
  }
  for (int i = 0; i < num; i++) {
    vector<double> temp;
    for (int j = 0; j < num; j++) {
      if (i == j)
        temp.push_back(inf);
      else
        temp.push_back(dist(points[i].first, points[i].second, points[j].first,
                            points[j].second));
    }
    distances.push_back(temp);
  }

  greedy(route, num);
  two_opt(route, num, begin_time);
  int ind = 0;
  bool fl = false;
  vector<int> res;
  fl = false;

  for (int i = 0; i < num; i++) {
    if (route[i] == 0) {
      ind = i;
      fl = true;
    }
    if (fl) res.push_back(route[i]);
  }
  for (int i = 0; i < ind; i++) res.push_back(route[i]);
  res.push_back(0);
  //    cout << calc_len() << '\n';
  // cout << fixed << setprecision(5) << calc_len() << '\n';
  return res;
}
// int main() {
//   clock_t begin_time = clock();
//   int num = 0;
//   double x, y;
//   cin >> num;
//   row points;
//   for (int i = 0; i < num; i++) {
//     cin >> x >> y;
//     points.push_back(make_pair(x, y));
//   }

//   for (int i = 0; i < num; i++) {
//     vector<double> temp;
//     for (int j = 0; j < num; j++) {
//       if (i == j)
//         temp.push_back(inf);
//       else
//         temp.push_back(dist(points[i].first, points[i].second,
//         points[j].first,
//                             points[j].second));
//     }
//     distances.push_back(temp);
//   }

//   greedy(route, num);

//   //    print_tour();

//   two_opt(route, num, begin_time);
//   if ((float)(clock() - begin_time) / CLOCKS_PER_SEC < 9.5)
//     three_opt(route, begin_time);

//   int ind = 0;
//   bool fl = false;

//   fl = false;
//   for (int i = 0; i < num; i++) {
//     if (route[i] == 0) {
//       ind = i;
//       fl = true;
//     }
//     if (fl) cout << route[i] + 1 << ' ';
//   }
//   for (int i = 0; i < ind; i++) cout << route[i] + 1 << ' ';
//   cout << 1 << '\n';
//   //    cout << calc_len() << '\n';
//   return 0;
// }
#endif