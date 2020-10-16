#include "astar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <unordered_map>
#include <variant>

#include <uat/slot.hpp>
#include <cool/compose.hpp>
#include <boost/functional/hash.hpp>

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

struct step {
  tslot first;
  tslot second;
  auto operator==(const step& other) const { return first == other.first && second == other.second; }
};

namespace std
{
template <> struct hash<step>
{
  auto operator()(const step& s) const noexcept -> size_t {
    size_t seed = 0;
    boost::hash_combine(seed, std::hash<tslot>{}(s.first));
    boost::hash_combine(seed, std::hash<tslot>{}(s.second));
    return seed;
  }
};
}

auto astar(const uat::slot& from, const uat::slot& to, uat::uint_t tstart,
           uat::value_t bid_max_value, uat::status_t& status, int seed) -> std::vector<uat::tslot>
{
  if (std::holds_alternative<unavailable>(status(from, tstart)))
    return {};

  std::mt19937 gen(seed);

  const auto h = [to, bid_max_value](const slot& s, uint_t t) -> value_t {
    return 2 * s.heuristic_distance(to) * bid_max_value;
  };

  const auto cost = [&](const tslot& s) {
    return std::visit(cool::compose{
      [&](unavailable) { return std::numeric_limits<value_t>::infinity(); },
      [&](owned) -> value_t { return 0; },
      [&](available status) {
        return status.min_value > bid_max_value ?
          std::numeric_limits<value_t>::infinity() :
          bid_max_value;
      }
    }, status(s.slot(), s.time()));
  };

  std::unordered_map<step, step> came_from;
  std::unordered_map<step, score_t> score;

  const step first = {{from, tstart}, {from, tstart}};
  score[first] = {0, h(from, tstart)};

  const auto cmp = [&score](const step& a, const step& b) {
    return score[a].f > score[b].f;
  };

  heap<step, decltype(cmp)> open(cmp);
  open.push(first);

  const auto try_path = [&](const step& current, step next) {
    const auto d = cost(next.first) + cost(next.second);
    if (std::isinf(d))
      return;

    const auto tentative = score[current].g + d;
    const auto hnext = h(next.second.slot(), next.second.time());

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

    if (current.second.slot() == to) {
      std::vector<tslot> path;
      auto s = current;
      for (; s.second.slot() != from; s = came_from.at(s)) {
        path.push_back(s.second);
        path.push_back(s.first);
      }
      path.push_back(s.second);

      return path;
    }

    // XXX: should we keep forbiding staying still?
    // try_path(current, {current.slot(), current.time() + 1});

    auto nei = current.second.slot().neighbors();
    std::shuffle(nei.begin(), nei.end(), gen);
    for (auto nslot : nei)
      try_path(current, {{current.second.slot(), current.second.time() + 1}, {std::move(nslot), current.second.time() + 1}});
  }

  return {};
}
