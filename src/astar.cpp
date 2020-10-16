#include "astar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <unordered_map>
#include <variant>

#include <uat/slot.hpp>
#include <cool/compose.hpp>

using namespace uat;

struct score_t {
  value_t g = std::numeric_limits<value_t>::infinity();
  value_t f = std::numeric_limits<value_t>::infinity();
};

template <typename T, typename Cmp>
class heap
{
public:
  heap(Cmp cmp) : cmp_(std::move(cmp)) {}

  auto push(T value)
  {
    for (auto& v : values_)
      if (v.first == value)
        v.second = false;
    values_.push_back({std::move(value), true});
    std::push_heap(values_.begin(), values_.end(), [&](const auto& x, const auto& y) {
      return cmp_(x.first, y.first);
    });
  }

  auto pop() -> std::pair<T, bool>
  {
    while (values_.size() > 1)
    {
      std::pop_heap(values_.begin(), values_.end(), [&](const auto& x, const auto& y) {
        return cmp_(x.first, y.first);
      });
      auto current = std::move(values_.back());
      values_.pop_back();
      if (current.second)
        return current;
    }
    auto value = std::move(values_.back());
    values_.pop_back();
    return value;
  }

  auto empty() const { return values_.empty(); }

private:
  std::vector<std::pair<T, bool>> values_;
  Cmp cmp_;
};

auto astar(const uat::slot& from, const uat::slot& to, uat::uint_t tstart,
           uat::value_t bid_max_value, uat::status_t& status, int seed) -> std::vector<uat::tslot>
{
  if (std::holds_alternative<unavailable>(status(from, tstart)))
    return {};

  std::mt19937 gen(seed);

  const auto h = [to, bid_max_value](const slot& s, uint_t t) -> value_t {
    return s.heuristic_distance(to) * bid_max_value;
  };

  const auto cost = [&](const tslot& s) {
    return std::visit(cool::compose{
      [&](unavailable) { return std::numeric_limits<value_t>::infinity(); },
      [&](owned) { return value_t{0}; },
      [&](available status) {
        return status.min_value > bid_max_value ?
          std::numeric_limits<value_t>::infinity() :
          bid_max_value;
      }
    }, status(s.slot(), s.time()));
  };

  std::unordered_map<tslot, tslot> came_from;
  std::unordered_map<tslot, score_t> score;

  score[{from, tstart}] = {0, h(from, tstart)};

  const auto cmp = [&score](const tslot& a, const tslot& b) {
    return score[a].f > score[b].f;
  };

  heap<tslot, decltype(cmp)> open(cmp);
  open.push({from, tstart});

  const auto try_path = [&](const tslot& current, tslot next) {
    const auto d = cost(next);
    if (std::isinf(d))
      return;

    const auto tentative = score[current].g + d;
    const auto hnext = h(next.slot(), next.time());

    if (tentative < score[next].g) {
      came_from[next] = current;
      score[next] = {tentative, tentative + hnext};

      open.push(std::move(next));
    }
  };

  while (!open.empty())
  {
    const auto [current, valid] = open.pop();
    if (!valid)
      continue;

    if (current.slot() == to) {
      std::vector<tslot> path;
      path.push_back(current);

      while (path.back().slot() != from)
        path.push_back(came_from.at(path.back()));

      return path;
    }

    // XXX: should we keep forbiding staying still?
    // try_path(current, {current.slot(), current.time() + 1});

    auto nei = current.slot().neighbors();
    std::shuffle(nei.begin(), nei.end(), gen);
    for (auto nslot : nei)
      try_path(current, {std::move(nslot), current.time() + 1});
  }

  return {};
}
