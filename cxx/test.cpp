#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include "BitmapSpace.hpp"
#include "Blossom.hpp"
#include "Euclidian.hpp"
#include "RRT.hpp"
#include "SpaceModifiers.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include <stb/stb_image.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb/stb_image_write.h>

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
  return ret;
}

std::optional<Path<TwoSpace::Reachability, TwoSpace::Config>>
singleTreeBlossomSearch() {
  // Poor seeds for MT here,
  // http://www.iro.umontreal.ca/~lecuyer/myftp/papers/lfsr04.pdf
  const uint32_t biasSeed = 0;
  const uint32_t spaceSeed = 1;

  const auto orthogonalMoves = [](const auto, const Direction,
                                  const TwoSpace::Config initial,
                                  const TwoSpace::Config)
      -> std::vector<std::pair<std::monostate, TwoSpace::Config>> {
    const float d = 1.f / 8;
    return std::vector<std::pair<std::monostate, TwoSpace::Config>>{
        {{}, initial + TwoSpace::Config{-d, 0}},
        {{}, initial + TwoSpace::Config{+d, 0}},
        {{}, initial + TwoSpace::Config{0, -d}},
        {{}, initial + TwoSpace::Config{0, +d}}};
  };
  Blossom<BiasedTwoSpace,
          std::vector<std::pair<std::monostate, TwoSpace::Config>>>
      twoSpace(orthogonalMoves, 0.1, biasSeed, spaceSeed);
  const TwoSpace::Config initial{0, 0};
  const TwoSpace::Config goal{1, 1};

  const auto ret = rrtSearch(0, twoSpace, 1000, initial, goal);
  return ret;
}

std::optional<Path<TwoSpace::Reachability, TwoSpace::Config>>
dualTreeBlossomSearch() {
  // Poor seeds for MT here,
  // http://www.iro.umontreal.ca/~lecuyer/myftp/papers/lfsr04.pdf
  const uint32_t biasSeed = 0;
  const uint32_t spaceSeed = 1;

  const auto orthogonalMoves = [](const auto, const Direction,
                                  const TwoSpace::Config initial,
                                  const TwoSpace::Config)
      -> std::vector<std::pair<std::monostate, TwoSpace::Config>> {
    const float d = 1.f / 8;
    return std::vector<std::pair<std::monostate, TwoSpace::Config>>{
        {{}, initial + TwoSpace::Config{-d, 0}},
        {{}, initial + TwoSpace::Config{+d, 0}},
        {{}, initial + TwoSpace::Config{0, -d}},
        {{}, initial + TwoSpace::Config{0, +d}}};
  };
  Blossom<BiasedTwoSpace,
          std::vector<std::pair<std::monostate, TwoSpace::Config>>>
      twoSpace(orthogonalMoves, 0.1, biasSeed, spaceSeed);
  const TwoSpace::Config initial{0, 0};
  const TwoSpace::Config goal{1, 1};

  const auto ret = rrtSearchDualTree(0, twoSpace, 1000, initial, goal);
  return ret;
}

std::optional<Path<BitmapSpace::Reachability, BitmapSpace::Config>>
bitmapSearch(const std::string &filename) {
  // Poor seeds for MT here,
  // http://www.iro.umontreal.ca/~lecuyer/myftp/papers/lfsr04.pdf
  const uint32_t biasSeed = 0;
  const uint32_t spaceSeed = 1;
  const uint32_t maxSampleAttempts = 100;
  BiasTowardsGoal<BitmapSpace> space(0.1, biasSeed, spaceSeed,
                                     maxSampleAttempts);
  space.load(filename);
  const BitmapSpace::Config initial{0, 0};
  const BitmapSpace::Config goal{space.getWidth() - 1, space.getHeight() - 1};

  const auto ret = rrtSearch(0, space, 1000, initial, goal);
  if (ret.has_value()) {
    space.saveWithPath("solved_" + filename, initial, ret.value());
  }
  return ret;
}

std::optional<Path<BitmapSpace::Reachability, BitmapSpace::Config>>
bitmapSearchBlossom(const std::string &filename) {
  // Poor seeds for MT here,
  // http://www.iro.umontreal.ca/~lecuyer/myftp/papers/lfsr04.pdf
  const uint32_t biasSeed = 0;
  const uint32_t spaceSeed = 1;
  const uint32_t maxSampleAttempts = 100;

  // These set of moves cast a ray in 8 directions, and return an edge which
  // travels to the intersection.
  // It also returns the result of casting a ray towards the target
  using Moves = std::vector<std::pair<std::monostate, BitmapSpace::Config>>;
  const auto move8 = [&](const BitmapSpace &space, const Direction,
                         const BitmapSpace::Config initial,
                         const BitmapSpace::Config target) -> Moves {
    Moves ret;
    ret.reserve(9);
    ret.push_back({{}, space.rayCast(initial, target)});
    for (int i = -1; i <= 1; ++i) {
      for (int j = -1; j <= 1; ++j) {
        if (i == 0 && j == 0) {
          continue;
        }
        const auto d = BitmapSpace::Config{1000 * i, 1000 * j};
        const auto hit = space.rayCast(initial, initial + d);
        ret.push_back({{}, hit});
      }
    }
    return ret;
  };
  Blossom<BiasTowardsGoal<BitmapSpace>, Moves> space(
      move8, 0.1, biasSeed, spaceSeed, maxSampleAttempts);

  space.load(filename);
  const BitmapSpace::Config initial{0, 0};
  const BitmapSpace::Config goal{space.getWidth() - 1, space.getHeight() - 1};

  const auto ret = rrtSearch(0, space, 2000, initial, goal);
  if (ret.has_value()) {
    space.saveWithPath("solved_" + filename, initial, ret.value());
  }
  return ret;
}

TEST_CASE("RRT Search Succeeds") {
  REQUIRE(singleTreeSearch().has_value());
  REQUIRE(dualTreeSearch().has_value());
  REQUIRE(singleTreeBlossomSearch().has_value());
  REQUIRE(dualTreeBlossomSearch().has_value());
  REQUIRE(bitmapSearch("images/clear.png").has_value());
  REQUIRE(bitmapSearch("images/wiggle.png").has_value());
  REQUIRE(!bitmapSearch("images/sad.png").has_value());
  REQUIRE(bitmapSearch("images/bottlenecks.png").has_value());
  REQUIRE(bitmapSearchBlossom("images/tunnel.png").has_value());
  REQUIRE(bitmapSearchBlossom("images/rooms.png").has_value());
}
