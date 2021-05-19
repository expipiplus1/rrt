#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include "Euclidian.hpp"
#include "RRT.hpp"
#include "SpaceModifiers.hpp"

using BiasedTwoSpace = BiasTowardsGoal<TwoSpace>;

std::optional<Path<TwoSpace::Reachability, TwoSpace::Config>>
singleTreeSearch() {
  // Poor seeds for MT here,
  // http://www.iro.umontreal.ca/~lecuyer/myftp/papers/lfsr04.pdf
  const uint32_t biasSeed = 0;
  const uint32_t spaceSeed = 1;
  BiasedTwoSpace twoSpace(0.1, biasSeed, spaceSeed);
  const TwoSpace::Config initial{0, 0};
  const TwoSpace::Config goal{1, 1};

  const auto ret = rrtSearch(0, twoSpace, 1000, initial, goal);
  for (const auto &[_, x] : ret.value()) {
    std::cout << x << std::endl;
  }
  return ret;
}

std::optional<Path<TwoSpace::Reachability, TwoSpace::Config>> dualTreeSearch() {
  // Poor seeds for MT here,
  // http://www.iro.umontreal.ca/~lecuyer/myftp/papers/lfsr04.pdf
  const uint32_t biasSeed = 0;
  const uint32_t spaceSeed = 1;
  BiasedTwoSpace twoSpace(0.1, biasSeed, spaceSeed);
  const TwoSpace::Config initial{0, 0};
  const TwoSpace::Config goal{1, 1};

  const auto ret = rrtSearchDualTree(0, twoSpace, 1000, initial, goal);
  for (const auto &[_, x] : ret.value()) {
    std::cout << x << std::endl;
  }
  return ret;
}

TEST_CASE("RRT Search Succeeds", "[rrtSearch]") {
  REQUIRE(singleTreeSearch().has_value());
  REQUIRE(dualTreeSearch().has_value());
}
