#include "fmt/core.h"
#include "hexgrid.hpp"
#include "naive.hpp"

#include <cool/indices.hpp>
#include <random>
#include <uat/simulation.hpp>

#include <CLI/CLI.hpp>
#include <variant>

int main(int argc, char* argv[])
{
  using namespace uat;

  CLI::App app{"Simulate a variation of 10.1109/ACCESS.2020.3030612."};

  struct
  {
    uint_t start_time = 100u;
    uint_t max_time = 100u;

    uint_t arrival_rate = 1u;

    std::array<int, 4u> dimensions = {15, 15, 3, 50}; // up to ~30k nodes in the network!

    int seed = -1;

    std::string pattern; // TODO: we are starting with the "free" network, not the traded permits
  } opts;

  app.add_option("-p,--start-print-time", opts.start_time, "Start time to print results");
  app.add_option("-t,--max-time", opts.max_time, "Simulation maximum time");
  app.add_option("-l,--arrival-rate", opts.arrival_rate, "Number of agents generated at each epoch");

  app.add_option("-d,--dimensions", opts.dimensions, "Airspace dimensions (X, Y, Z, T)");

  app.add_option("-s,--seed", opts.seed, "Random seed (random_device if < 0)");

  app.add_option("-n,--network-data", opts.pattern, "Pattern of the network filename");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  assert(opts.dimensions[3] > 1);

  auto factory = [&](uint_t t, const airspace& space, int seed) -> std::vector<agent> {
    std::mt19937 rng(seed);

    std::vector<agent> result;
    result.reserve(opts.arrival_rate);
    for ([[maybe_unused]] const auto _ : cool::indices(opts.arrival_rate))
      result.push_back(Naive(space, opts.dimensions[3], rng()));

    return result;
  };

  // TODO: consider ground for other times
  const auto status_callback = [start_time = opts.start_time, time_window = opts.dimensions[3]](
                                 uint_t t, uint_t nactive, const airspace&, permit_private_status_fn) {
    fmt::print(stderr, "t = {}\n", t);
    fmt::print(stdout, "{}\n", nactive);
    // if (t < start_time)
    //   return;
    // using namespace permit_private_status;
    // const auto end = t + time_window;
    // ++t;

    // std::unordered_set<region> current, next;
    // space.iterate([&](region location) -> bool {
    //   if (location.downcast<HexRegion>().altitude() != 0u)
    //     return true;
    //   if (std::holds_alternative<on_sale>(status(location, t)))
    //     current.insert(location);
    //   return true;
    // });

    // fmt::print("t,xa,ya,za,xb,yb,zb\n");
    // for (; t < end && !current.empty(); ++t) {
    //   for (const auto& location : current) {
    //     for (const auto& adj : location.adjacent_regions()) {
    //       if (!std::holds_alternative<on_sale>(status(adj, t + 1)))
    //         continue;
    //       if (adj.downcast<HexRegion>().altitude() > end - t - 1)
    //         continue;
    //       fmt::print("{},{},{}\n", t, location, adj);
    //       next.insert(adj);
    //     }
    //   }
    //   current = std::move(next);
    //   next.clear();
    // }
  };

  simulation_opts_t sopts = {.time_window = opts.dimensions[3],
                             .stop_criteria = stop_criteria::time_threshold_t{opts.max_time},
                             .status_callback = status_callback};

  simulate(factory, HexGrid{{opts.dimensions[0], opts.dimensions[1], opts.dimensions[2]}},
           opts.seed < 0 ? std::random_device{}() : opts.seed, sopts);
}
