#include <algorithm>
#include <array>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "IOutils.hpp"
struct GridLocation {
  int x, y;
};
namespace std {
/* implement hash function so we can put GridLocation into an unordered_set */
template <>
struct hash<GridLocation> {
  typedef GridLocation argument_type;
  typedef std::size_t result_type;
  std::size_t operator()(const GridLocation& id) const noexcept {
    return std::hash<int>()(id.x ^ (id.y << 4));
  }
};
}  // namespace std
struct SquareGrid {
  static std::array<GridLocation, 4> DIRS;

  int width, height;
  double nodata;
  std::vector<std::vector<double> > RasterData;
  double step;
  SquareGrid(ProjectionDataset Map) {
    nodata = Map.Nodata;
    width = Map.sizeX;
    height = Map.sizeY;
    RasterData = Map.RasterData;
    step = Map.step;
  }
  bool in_bounds(GridLocation id) const {
    return 0 <= id.x && id.x < width && 0 <= id.y && id.y < height;
  }
  bool passable(GridLocation id) const {
    return fabs(RasterData[id.x][id.y] - nodata) > 1e-5;
  }

  std::vector<GridLocation> neighbors(GridLocation id) const {
    std::vector<GridLocation> results;

    for (GridLocation dir : DIRS) {
      GridLocation next{id.x + dir.x, id.y + dir.y};
      if (in_bounds(next) && passable(next)) {
        results.push_back(next);
      }
    }

    if ((id.x + id.y) % 2 == 0) {
      // see "Ugly paths" section for an explanation:
      std::reverse(results.begin(), results.end());
    }

    return results;
  }
  double cost(GridLocation from_node, GridLocation to_node) const {
    return step + abs(ceil(RasterData[from_node.x][from_node.y]) -
                      ceil(RasterData[to_node.x][to_node.y]));
  }
  inline double heuristic(GridLocation a, GridLocation b) {
    return step * (std::abs(a.x - b.x) + std::abs(a.y - b.y)) +
           fabs(ceil(RasterData[a.x][a.y]) - ceil(RasterData[b.x][b.y]));
  }
};
std::array<GridLocation, 4> SquareGrid::DIRS = {
    /* East, West, North, South */
    GridLocation{1, 0}, GridLocation{-1, 0}, GridLocation{0, -1},
    GridLocation{0, 1}};

// Helpers for GridLocation

bool operator==(GridLocation a, GridLocation b) {
  return a.x == b.x && a.y == b.y;
}

bool operator!=(GridLocation a, GridLocation b) { return !(a == b); }

bool operator<(GridLocation a, GridLocation b) {
  return std::tie(a.x, a.y) < std::tie(b.x, b.y);
}

template <typename T, typename priority_t>
struct PriorityQueue {
  typedef std::pair<priority_t, T> PQElement;
  std::priority_queue<PQElement, std::vector<PQElement>,
                      std::greater<PQElement> >
      elements;

  inline bool empty() const { return elements.empty(); }

  inline void put(T item, priority_t priority) {
    elements.emplace(priority, item);
  }

  T get() {
    T best_item = elements.top().second;
    elements.pop();
    return best_item;
  }
};

template <typename Location>
std::vector<Location> reconstruct_path(
    Location start, Location goal,
    std::unordered_map<Location, Location> came_from) {
  std::vector<Location> path;
  Location current = goal;
  while (current != start) {
    path.push_back(current);
    current = came_from[current];
  }
  path.push_back(start);  // optional
  std::reverse(path.begin(), path.end());
  return path;
}

template <typename Location, typename Graph>
void a_star_search(Graph graph, Location start, Location goal,
                   std::unordered_map<Location, Location>& came_from,
                   std::unordered_map<Location, double>& cost_so_far) {
  PriorityQueue<Location, double> frontier;
  frontier.put(start, 0);

  came_from[start] = start;
  cost_so_far[start] = 0;

  while (!frontier.empty()) {
    Location current = frontier.get();

    if (current == goal) {
      break;
    }
    for (Location next : graph.neighbors(current)) {
      double new_cost = cost_so_far[current] + graph.cost(current, next);
      if (cost_so_far.find(next) == cost_so_far.end() ||
          new_cost < cost_so_far[next]) {
        cost_so_far[next] = new_cost;
        double priority = new_cost;  // graph.heuristic(next, goal);
        frontier.put(next, priority);
        came_from[next] = current;
      }
    }
  }
}
template <class Graph>
void draw_grid(
    const Graph& graph,
    std::unordered_map<GridLocation, double>* distances = nullptr,
    std::unordered_map<GridLocation, GridLocation>* point_to = nullptr,
    std::vector<GridLocation>* path = nullptr, GridLocation* start = nullptr,
    GridLocation* goal = nullptr) {
  const int field_width = 3;
  std::cout << std::string(field_width * graph.width, '_') << '\n';
  for (int y = 0; y != graph.height; ++y) {
    for (int x = 0; x != graph.width; ++x) {
      GridLocation id{x, y};
      if (start && id == *start) {
        std::cout << " A ";
      } else if (goal && id == *goal) {
        std::cout << " Z ";
      } else if (path != nullptr &&
                 find(path->begin(), path->end(), id) != path->end()) {
        std::cout << " @ ";
      } else if (point_to != nullptr && point_to->count(id)) {
        GridLocation next = (*point_to)[id];
        if (next.x == x + 1) {
          std::cout << " > ";
        } else if (next.x == x - 1) {
          std::cout << " < ";
        } else if (next.y == y + 1) {
          std::cout << " v ";
        } else if (next.y == y - 1) {
          std::cout << " ^ ";
        } else {
          std::cout << " * ";
        }
      } else if (distances != nullptr && distances->count(id)) {
        std::cout << ' ' << std::left << std::setw(field_width - 1)
                  << (*distances)[id];
      } else {
        std::cout << " . ";
      }
    }
    std::cout << '\n';
  }
  std::cout << std::string(field_width * graph.width, '~') << '\n';
}