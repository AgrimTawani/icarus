#include "local_planner/local_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace icarus::local_planner {
namespace {

double PointSegmentDistance(double px, double py, double ax, double ay,
                            double bx, double by) {
  const double dx = bx - ax;
  const double dy = by - ay;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared == 0.0) return std::hypot(px - ax, py - ay);
  const double t = std::clamp(((px - ax) * dx + (py - ay) * dy) /
                                  length_squared,
                              0.0, 1.0);
  return std::hypot(px - (ax + t * dx), py - (ay + t * dy));
}

}  // namespace

LocalPlanner::LocalPlanner(double resolution_m, double clearance_m,
                           double planning_margin_m,
                           std::uint32_t maximum_cells)
    : resolution_m_(resolution_m),
      clearance_m_(clearance_m),
      margin_m_(planning_margin_m),
      maximum_cells_(maximum_cells) {}

bool LocalPlanner::SegmentClear(
    const v1::LocalPositionNed& start, const v1::LocalPositionNed& goal,
    const std::vector<perception::Point3>& obstacles,
    double requested_clearance_m) const {
  const double clearance_m = std::max(clearance_m_, requested_clearance_m);
  for (const auto& point : obstacles) {
    if (std::abs(point.z_m - start.down_m()) > 2.0) continue;
    if (PointSegmentDistance(point.x_m, point.y_m, start.north_m(),
                             start.east_m(), goal.north_m(), goal.east_m()) <
        clearance_m) {
      return false;
    }
  }
  return true;
}

PlanResult LocalPlanner::Plan(
    const v1::LocalPositionNed& start, const v1::LocalPositionNed& goal,
    const std::vector<perception::Point3>& obstacles,
    double requested_clearance_m, double departure_clearance_floor_m) const {
  PlanResult result;
  const double clearance_m = std::max(clearance_m_, requested_clearance_m);
  // A caller may distinguish the live clearance floor from a soft planning
  // margin. Only the departure edge may leave that margin, and every nearby
  // return must get no closer; its endpoint must regain the full envelope.
  const auto departure_clear = [&](const v1::LocalPositionNed& to) {
    if (!(departure_clearance_floor_m > 0.0) ||
        departure_clearance_floor_m > clearance_m) return false;
    for (const auto& point : obstacles) {
      if (std::abs(point.z_m - start.down_m()) > 2.0) continue;
      const double distance = std::hypot(start.north_m() - point.x_m,
                                         start.east_m() - point.y_m);
      if (distance < departure_clearance_floor_m) return false;
      if (std::hypot(to.north_m() - point.x_m, to.east_m() - point.y_m) < clearance_m)
        return false;
      if (distance < clearance_m) {
        const double outward = (start.north_m() - point.x_m) * (to.north_m() - start.north_m()) +
                               (start.east_m() - point.y_m) * (to.east_m() - start.east_m());
        if (outward < 0.0) return false;
      } else if (PointSegmentDistance(point.x_m, point.y_m, start.north_m(),
                   start.east_m(), to.north_m(), to.east_m()) < clearance_m) return false;
    }
    return true;
  };
  if (SegmentClear(start, goal, obstacles, clearance_m) || departure_clear(goal)) {
    result.success = true;
    result.direct_path = true;
    result.reason = "direct path clear";
    result.waypoints.push_back(goal);
    return result;
  }

  const double min_n = std::min(start.north_m(), goal.north_m()) - margin_m_;
  const double max_n = std::max(start.north_m(), goal.north_m()) + margin_m_;
  const double min_e = std::min(start.east_m(), goal.east_m()) - margin_m_;
  const double max_e = std::max(start.east_m(), goal.east_m()) + margin_m_;
  const int width = static_cast<int>(std::ceil((max_n - min_n) / resolution_m_)) + 1;
  const int height = static_cast<int>(std::ceil((max_e - min_e) / resolution_m_)) + 1;
  const int cell_count = width * height;
  if (width <= 0 || height <= 0 || cell_count <= 0 ||
      static_cast<std::uint32_t>(cell_count) > maximum_cells_) {
    result.reason = "planning window exceeds configured bound";
    return result;
  }
  const auto index = [width](int x, int y) { return y * width + x; };
  const auto coordinate = [&](double n, double e) {
    return std::pair<int, int>{
        std::clamp(static_cast<int>(std::lround((n - min_n) / resolution_m_)), 0,
                   width - 1),
        std::clamp(static_cast<int>(std::lround((e - min_e) / resolution_m_)), 0,
                   height - 1)};
  };
  std::vector<bool> occupied(cell_count, false);
  const int inflation = static_cast<int>(std::ceil(clearance_m / resolution_m_));
  for (const auto& point : obstacles) {
    if (std::abs(point.z_m - start.down_m()) > 2.0 || point.x_m < min_n ||
        point.x_m > max_n || point.y_m < min_e || point.y_m > max_e) continue;
    const auto [ox, oy] = coordinate(point.x_m, point.y_m);
    for (int dy = -inflation; dy <= inflation; ++dy) {
      for (int dx = -inflation; dx <= inflation; ++dx) {
        const int x = ox + dx;
        const int y = oy + dy;
        if (x >= 0 && x < width && y >= 0 && y < height &&
            std::hypot(dx * resolution_m_, dy * resolution_m_) <= clearance_m) {
          occupied[index(x, y)] = true;
        }
      }
    }
  }
  const auto [sx, sy] = coordinate(start.north_m(), start.east_m());
  const auto [gx, gy] = coordinate(goal.north_m(), goal.east_m());
  occupied[index(sx, sy)] = false;
  if (occupied[index(gx, gy)]) {
    result.reason = "destination intersects inflated obstacle";
    return result;
  }

  // Spatial buckets keep exact edge checks local rather than scanning every
  // LiDAR return for each of up to eight neighbors of every search cell.
  std::vector<std::vector<const perception::Point3*>> buckets(cell_count);
  for (const auto& point : obstacles) {
    if (point.x_m < min_n - clearance_m || point.x_m > max_n + clearance_m ||
        point.y_m < min_e - clearance_m || point.y_m > max_e + clearance_m) continue;
    const auto [x, y] = coordinate(point.x_m, point.y_m);
    buckets[index(x, y)].push_back(&point);
  }
  const auto edge_clear = [&](const v1::LocalPositionNed& from,
                              const v1::LocalPositionNed& to) {
    const auto [lo_x, lo_y] = coordinate(
        std::min(from.north_m(), to.north_m()) - clearance_m,
        std::min(from.east_m(), to.east_m()) - clearance_m);
    const auto [hi_x, hi_y] = coordinate(
        std::max(from.north_m(), to.north_m()) + clearance_m,
        std::max(from.east_m(), to.east_m()) + clearance_m);
    for (int y = lo_y; y <= hi_y; ++y) {
      for (int x = lo_x; x <= hi_x; ++x) {
        for (const auto* point : buckets[index(x, y)]) {
          if (std::abs(point->z_m - from.down_m()) <= 2.0 &&
              PointSegmentDistance(point->x_m, point->y_m,
                  from.north_m(), from.east_m(), to.north_m(), to.east_m()) < clearance_m)
            return false;
        }
      }
    }
    return true;
  };
  const auto position = [&](int cell) {
    if (cell == index(sx, sy)) return start;
    if (cell == index(gx, gy)) return goal;
    auto point = goal;
    point.set_north_m(min_n + (cell % width) * resolution_m_);
    point.set_east_m(min_e + (cell / width) * resolution_m_);
    return point;
  };

  struct QueueItem { double score; int cell; };
  struct Greater { bool operator()(const QueueItem& a, const QueueItem& b) const {
    return a.score > b.score;
  }};
  std::priority_queue<QueueItem, std::vector<QueueItem>, Greater> open;
  std::vector<double> cost(cell_count, std::numeric_limits<double>::infinity());
  std::vector<int> parent(cell_count, -1);
  const int start_cell = index(sx, sy);
  const int goal_cell = index(gx, gy);
  cost[start_cell] = 0.0;
  open.push({0.0, start_cell});
  // An existing clearance violation can occupy several grid cells. A
  // one-cell departure cannot reach the ordinary free-space graph. Seed
  // bounded free cells using the same exact, monotonic departure predicate;
  // all subsequent edges still require the full planning envelope.
  if (departure_clearance_floor_m > 0.0 &&
      departure_clearance_floor_m <= clearance_m &&
      !SegmentClear(start, start, obstacles, clearance_m)) {
    const int radius = static_cast<int>(std::min(
        static_cast<double>(std::max(width, height)),
        std::ceil(clearance_m / resolution_m_) + 1.0));
    for (int y = std::max(0, sy-radius); y <= std::min(height-1, sy+radius); ++y) {
      for (int x = std::max(0, sx-radius); x <= std::min(width-1, sx+radius); ++x) {
        const int cell = index(x, y);
        if (cell == start_cell || occupied[cell]) continue;
        const auto endpoint = position(cell);
        if (!departure_clear(endpoint)) continue;
        cost[cell] = std::hypot(endpoint.north_m()-start.north_m(),
                                endpoint.east_m()-start.east_m()) / resolution_m_;
        parent[cell] = start_cell;
        open.push({cost[cell] + std::hypot(gx-x, gy-y), cell});
      }
    }
  }
  constexpr std::array<std::pair<int, int>, 8> moves{{
      {-1, -1}, {0, -1}, {1, -1}, {-1, 0},
      {1, 0}, {-1, 1}, {0, 1}, {1, 1}}};
  while (!open.empty()) {
    const int cell = open.top().cell;
    open.pop();
    if (cell == goal_cell) break;
    const int x = cell % width;
    const int y = cell / width;
    for (const auto& [dx, dy] : moves) {
      const int nx = x + dx;
      const int ny = y + dy;
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
      const int next = index(nx, ny);
      if (occupied[next]) continue;
      const double candidate = cost[cell] + std::hypot(dx, dy);
      if (candidate >= cost[next]) continue;
      // Free endpoints do not imply a clear diagonal or off-grid connection.
      // Use the actual start/goal poses, not their rounded grid centers.
      if (!edge_clear(position(cell), position(next)) &&
          !(cell == start_cell && departure_clear(position(next)))) continue;
      cost[next] = candidate;
      parent[next] = cell;
      open.push({candidate + std::hypot(gx - nx, gy - ny), next});
    }
  }
  if (parent[goal_cell] < 0) {
    result.reason = "no collision-free path in bounded local map";
    return result;
  }
  std::vector<int> cells;
  for (int cell = goal_cell; cell != start_cell; cell = parent[cell]) {
    cells.push_back(cell);
  }
  std::reverse(cells.begin(), cells.end());
  std::vector<v1::LocalPositionNed> full_path;
  full_path.reserve(cells.size());
  for (const int cell : cells) full_path.push_back(position(cell));
  v1::LocalPositionNed anchor = start;
  std::size_t cursor = 0;
  while (cursor < full_path.size()) {
    std::size_t farthest = full_path.size();
    for (std::size_t candidate = full_path.size(); candidate-- > cursor;) {
      if (SegmentClear(anchor, full_path[candidate], obstacles, clearance_m) ||
          (cursor == 0 && departure_clear(full_path[candidate]))) {
        farthest = candidate;
        break;
      }
    }
    if (farthest == full_path.size()) {
      result.waypoints.clear();
      result.reason = "no clear connection while simplifying local route";
      return result;
    }
    result.waypoints.push_back(full_path[farthest]);
    anchor = full_path[farthest];
    cursor = farthest + 1;
  }
  result.success = !result.waypoints.empty();
  result.reason = result.success ? "collision-free detour generated"
                                 : "planner produced an empty route";
  return result;
}

}  // namespace icarus::local_planner
