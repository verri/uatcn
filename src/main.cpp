// #include "airspace3d.hpp"
// #include "naive.hpp"
//
// #include <cstdio>
#include <uat/simulation.hpp>
// #include <cool/indices.hpp>
// #include <cool/ccreate.hpp>
// #include <random>

#include <CLI/CLI.hpp>

int main(int argc, char *argv[])
{
  using namespace uat;

  CLI::App app{"Simulate a variation of 10.1109/ACCESS.2020.3030612."};

  struct
  {
    uint_t max_time = 1000;
    uint_t arrival_rate = 1;

    std::array<uint_t, 4> dimensions = {50, 50, 5, 100};

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

  // const auto open_file = [](const std::string& filename) -> std::FILE* {
  //   if (filename.empty())
  //     return nullptr;
  //   return filename == "-" ? stdout : std::fopen(filename.c_str(), "w");
  // };

  // const auto safe_close = [](std::FILE *fp) {
  //   if (fp && fp != stdout)
  //     std::fclose(fp);
  // };

  // const auto afile = cool::ccreate(open_file(opts.afilename), safe_close);
  // if (afile)
  //   fmt::print(afile.get(), "Id,StartTime,Iterations,CongestionParam,FromX,FromY,FromZ,ToX,ToY,ToZ,Fundamental,Sigma,MinDistance,Distance\n");

  // const auto pfile = cool::ccreate(open_file(opts.pfilename), safe_close);
  // if (pfile)
  //   fmt::print(pfile.get(), "Id,X,Y,Z,Time\n");

  // const auto tfile = cool::ccreate(open_file(opts.tfilename), safe_close);
  // if (tfile)
  //   fmt::print(tfile.get(), "TransactionTime,From,To,X,Y,Z,Time,Value\n");

  // auto factory = [&](uint_t t, const airspace& space, int seed) -> std::vector<agent> {
  //   if (t >= opts.max_time)
  //     return {};

  //   std::mt19937 rng(seed);

  //   std::vector<agent> result;
  //   result.reserve(opts.n_agents);
  //   for ([[maybe_unused]] const auto _ : cool::indices(opts.n_agents))
  //     result.push_back(Naive(space, rng(), afile.get(), pfile.get()));

  //   return result;
  // };

  // simulate(factory, Airspace3D{opts.dimensions},
  //     opts.seed < 0 ? std::random_device{}() : opts.seed,
  //     tfile.get());
}
