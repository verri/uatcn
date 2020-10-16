#include "hexgrid.hpp"
#include "naive.hpp"

#include <uat/simulation.hpp>
#include <cool/indices.hpp>
#include <random>

#include <CLI/CLI.hpp>

int main(int argc, char *argv[])
{
  using namespace uat;

  CLI::App app{"Simulate a variation of 10.1109/ACCESS.2020.3030612."};

  struct
  {
    uint_t max_time = 100;
    uint_t arrival_rate = 1;

    std::array<int, 4> dimensions = {100, 100, 10, 300};

    int seed = -1;

    std::string pattern; // TODO: we are starting with the "free" network, not the traded permits
  } opts;

  app.add_option("-t,--max-time", opts.max_time, "Simulation maximum time");
  app.add_option("-l,--arrival-rate", opts.arrival_rate, "Number of agents generated at each epoch");

  app.add_option("-d,--dimensions", opts.dimensions, "Airspace dimensions (X, Y, Z, T)");

  app.add_option("-s,--seed", opts.seed, "Random seed (random_device if < 0)");

  app.add_option("-n,--network-data", opts.pattern, "Pattern of the network filename");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError &e) {
    return app.exit(e);
  }

  assert(opts.dimensions[3] > 1);

  auto factory = [&](uint_t t, const airspace& space, int seed) -> std::vector<agent> {
    if (t >= opts.max_time)
      return {};

    std::mt19937 rng(seed);

    std::vector<agent> result;
    result.reserve(opts.arrival_rate);
    for ([[maybe_unused]] const auto _ : cool::indices(opts.arrival_rate))
      result.push_back(Naive(space, opts.dimensions[3], rng()));

    return result;
  };

  simulate(
      factory,
      HexGrid{{ opts.dimensions[0], opts.dimensions[1], opts.dimensions[2] }},
      opts.seed < 0 ? std::random_device{}() : opts.seed,
      [](trade_info_t info){
        std::cout << info.value << '\n';
      });
}
