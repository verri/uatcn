#pragma once

#include <limits>
#include <uat/type.hpp>
#include <uat/airspace.hpp>
#include <uat/agent.hpp>
#include <uat/slot.hpp>

#include <unordered_set>
#include <cstdio>

// Exactly like 10.1109/ACCESS.2020.3030612
class Naive
{
public:
  Naive(const uat::airspace&, uat::uint_t, int);

  auto act(uat::uint_t, uat::bid_t, uat::status_t, int) -> bool;

  auto after_auction(uat::uint_t, uat::ask_t, uat::status_t, int) -> void;

  auto on_bought(const uat::slot&, uat::uint_t, uat::value_t) -> void;

private:
  uat::mission_t mission_;

  uat::value_t fundamental_;
  uat::value_t sigma_;

  uat::uint_t congestion_param_ = 1;
  uat::uint_t max_congestion_param_;

  uat::uint_t last_time_ = std::numeric_limits<uat::uint_t>::max();

  std::unordered_set<uat::tslot> keep_, onsale_;
};
