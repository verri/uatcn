#include "anxious.hpp"
#include "hexgrid.hpp"

#include <CLI/CLI.hpp>
#include <cool/ccreate.hpp>
#include <cool/channel.hpp>
#include <cool/indices.hpp>
#include <cstdio>
#include <fmt/core.h>
#include <fmt/format.h>
#include <random>
#include <thread>
#include <tuple>
#include <uat/simulation.hpp>
#include <variant>
#include <zlib.h>

static void compress_to(fmt::memory_buffer buf, std::string filename)
{
  const auto file = cool::ccreate(gzopen(filename.c_str(), "wb"), gzclose);
  gzwrite(file.get(), buf.data(), buf.size());
}

int main(int argc, char* argv[])
{
  using namespace uat;

  CLI::App app{"Simulate a variation of <https://doi.org/10.1109/ACCESS.2020.3030612>"};

  struct
  {
    uint_t start_time = 100u;
    uint_t max_time = 100u;

    uint_t arrival_rate = 1u;

    std::array<int, 4u> dimensions = {15, 15, 3, 23}; // up to ~15k nodes in the network!

    int seed = -1;

    std::string pattern;
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
      result.push_back(Anxious(space, opts.dimensions[3], rng()));

    return result;
  };

  cool::channel<std::tuple<fmt::memory_buffer, std::string>> ch;

  std::thread print_thr([&] {
    std::tuple<fmt::memory_buffer, std::string> args;
    while (ch >> args)
      std::apply(compress_to, std::move(args));
  });

  // TODO: consider ground for other times
  const auto status_callback = [start_time = opts.start_time, time_window = opts.dimensions[3], pattern = opts.pattern,
                                &ch](uint_t t, const agents_private_status_fn& agents, const airspace& space,
                                     permit_private_status_fn status) mutable {
    fmt::print(stderr, "{},{}\n", t, agents.active_count());
    if (pattern.empty() || t < start_time)
      return;

    using namespace permit_private_status;
    const auto curtime = t;
    const auto end = t + time_window;
    ++t;

    std::unordered_set<region> current, next;
    space.iterate([&](region location) -> bool {
      if (location.downcast<HexRegion>().altitude() != 0u)
        return true;
      if (std::holds_alternative<on_sale>(status(location, t)) && std::holds_alternative<on_sale>(status(location, t + 1)))
        current.insert(location);
      return true;
    });

    fmt::memory_buffer out;
    fmt::format_to(out, "t,xa,ya,za,xb,yb,zb\n");
    for (; t < end && !current.empty(); ++t) {
      for (const auto& location : current) {
        for (const auto& adj : location.adjacent_regions()) {
          if (!std::holds_alternative<on_sale>(status(adj, t + 1)) || !std::holds_alternative<on_sale>(status(adj, t + 2)))
            continue;
          if (adj.downcast<HexRegion>().altitude() > end - t - 1)
            continue;
          fmt::format_to(out, "{},{},{}\n", t, location, adj);
          next.insert(adj);
        }
      }
      std::swap(current, next);
      next.clear();
    }
    ch << std::forward_as_tuple(std::move(out), fmt::format(pattern, fmt::arg("time", curtime)));
  };

  simulation_opts_t sopts = {.time_window = opts.dimensions[3],
                             .stop_criteria = stop_criteria::time_threshold_t{opts.max_time},
                             .status_callback = status_callback};

  simulate(factory, HexGrid{{opts.dimensions[0], opts.dimensions[1], opts.dimensions[2]}},
           opts.seed < 0 ? std::random_device{}() : opts.seed, sopts);

  ch.close();
  print_thr.join();
}
