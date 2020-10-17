#include "naive.hpp"

#include "astar.hpp"

#include <cool/compose.hpp>
#include <cool/indices.hpp>
#include <jules/base/numeric.hpp>

#include <cassert>
#include <cstdio>
#include <iterator>
#include <limits>
#include <random>
#include <unordered_set>
#include <utility>
#include <variant>

using namespace uat;

Naive::Naive(const airspace& space, uint_t T, int seed)
{
  std::mt19937 rng(seed);

  std::uniform_real_distribution<value_t> f{50.0, 150.0};
  std::uniform_real_distribution<value_t> s{0.0, 0.2};

  fundamental_ = f(rng);
  sigma_ = s(rng);

  mission_ = space.random_mission(rng());

  const auto mission_length = mission_.length();
  assert(T > mission_length);

  max_congestion_param_ = T - mission_length;
}

auto Naive::act(uint_t t, uat::bid_fn bid, uat::permit_public_status_fn status, int seed) -> bool
{
  std::mt19937 rng(seed);
  std::normal_distribution<value_t> dist{0.0, sigma_ * fundamental_};

  onsale_ = std::exchange(keep_, {});

  // check previous path
  if (last_time_ != std::numeric_limits<uat::uint_t>::max()) {
    auto path = astar(mission_.from, mission_.to, last_time_, std::numeric_limits<value_t>::infinity(), status, rng());

    // mission completed
    if (path.size() != 0) {
      for (auto& position : path) {
        onsale_.erase(position);
        keep_.insert(std::move(position));
      }
      return false;
    }
  }

  const auto path = [&]() -> std::vector<permit> {
    uint_t start = 1u;
    std::vector<uint_t> tries;

    while (true) {
      tries.clear();
      tries.reserve(congestion_param_ - start + 1);

      ranges::copy(cool::closed_indices(start, congestion_param_), ranges::back_inserter(tries));
      ranges::shuffle(tries, rng);

      for (const auto wait : tries) {
        last_time_ = t + wait;
        auto p = astar(mission_.from, mission_.to, last_time_, fundamental_, status, rng());
        if (p.size() > 0) {
          congestion_param_ *= 2;
          return p;
        }
      }

      start = congestion_param_ + 1;
      congestion_param_ = std::min(max_congestion_param_, 2 * congestion_param_);
      if (start > congestion_param_)
        return {};
    }
  }();

  if (path.size() == 0)
    return true;

  for (const auto& [slot, t] : path) {
    using namespace permit_public_status;
    std::visit(cool::compose{
                 [](unavailable) { assert(false); },
                 [&, slot = slot, t = t](owned) {
                   onsale_.erase({slot, t});
                   keep_.emplace(std::move(slot), t);
                 },
                 [&, slot = slot, t = t](available) { bid(std::move(slot), t, fundamental_ - std::abs(dist(rng))); },
               },
               status(slot, t));
  }

  return keep_.size() != path.size(); // true if there are missing permits
}

auto Naive::after_trading(uint_t, ask_fn ask, permit_public_status_fn, int) -> void
{
  for (const auto& position : onsale_)
    ask(position.location(), position.time(), 0.0);
  onsale_.clear();
}

auto Naive::on_bought(const region& s, uint_t t, value_t) -> void { keep_.insert({s, t}); }
