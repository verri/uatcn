#pragma once

#include <limits>
#include <uat/agent.hpp>
#include <uat/airspace.hpp>
#include <uat/permit.hpp>
#include <uat/type.hpp>

#include <cstdio>
#include <unordered_set>

// Exactly like 10.1109/ACCESS.2020.3030612
class Anxious
{
public:
  Anxious(const uat::airspace&, uat::uint_t, int);

  auto act(uat::uint_t, uat::bid_fn, uat::permit_public_status_fn, int) -> bool;

  auto after_trading(uat::uint_t, uat::ask_fn, uat::permit_public_status_fn, int) -> void;

  auto on_bought(const uat::region&, uat::uint_t, uat::value_t) -> void;

private:
  uat::mission_t mission_;
  uat::value_t fundamental_;
  std::unordered_set<uat::permit> keep_, onsale_;
};
