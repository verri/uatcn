#include "naive.hpp"

#include "astar.hpp"

#include <cmath>
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

Naive::Naive(const airspace& space, [[maybe_unused]] uint_t T, int seed) // TODO
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<value_t> f{1e-6, 1.0};
  fundamental_ = f(rng);
  mission_ = space.random_mission(rng());
  assert(T > mission_.length());
}

auto Naive::act(uint_t t, uat::bid_fn bid, uat::permit_public_status_fn status, int seed) -> bool
{
  std::mt19937 rng(seed);
  onsale_ = std::exchange(keep_, {});

  {
    auto path = astar(mission_.from, mission_.to, t, std::numeric_limits<value_t>::infinity(), status, rng());
    // mission completed
    if (path.size() != 0) {
      for (auto& position : path) {
        onsale_.erase(position);
        keep_.insert(std::move(position));
      }
      return false;
    }
  }

  const auto path = astar(mission_.from, mission_.to, t + 1, fundamental_, status, rng());
  if (path.size() == 0) {
    fundamental_ *= 2;
    return true;
  }

  for (const auto& [slot, t] : path) {
    using namespace permit_public_status;
    std::visit(cool::compose{
                 [](unavailable) { assert(false); },
                 [&, slot = slot, t = t](owned) {
                   onsale_.erase({slot, t});
                   keep_.emplace(std::move(slot), t);
                 },
                 [&, slot = slot, t = t](available) { bid(std::move(slot), t, fundamental_); },
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
