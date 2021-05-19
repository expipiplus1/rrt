#include <array>
#include <cstdint>
#include <deque>
#include <variant>
#include <vector>

#include "BitmapSpace.hpp"
#include "Blossom.hpp"
#include "RRT.hpp"
#include "SpaceModifiers.hpp"

// I is an input_iterator
template <typename I>
std::vector<std::array<float, 2>> rrt(int width, int height, I dataBegin,
                                      I dataEnd,
                                      const std::array<float, 2> initialConfig,
                                      const std::array<float, 2> goalConfig) {
  // Poor seeds for MT here,
  // http://www.iro.umontreal.ca/~lecuyer/myftp/papers/lfsr04.pdf
  const uint32_t biasSeed = 0;
  const uint32_t spaceSeed = 1;
  const uint32_t maxSampleAttempts = 100;

  // These set of moves cast a ray in the 8 ordinal and cardinal directions and
  // return an edge which travels to the intersection. It also returns the
  // result of casting a ray towards the target
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

  space.loadFromMemory(width, height, dataBegin, dataEnd);

  // TODO: clamp these to the image boundaries, or throw if out of bounds.
  const Eigen::Vector2f imageSize{static_cast<float>(space.getWidth() - 1),
                                  static_cast<float>(space.getHeight() - 1)};
  const BitmapSpace::Config initial =
      imageSize
          .cwiseProduct(Eigen::Vector2f{initialConfig[0], initialConfig[1]})
          .cast<int>();
  const BitmapSpace::Config goal =
      imageSize.cwiseProduct(Eigen::Vector2f{goalConfig[0], goalConfig[1]})
          .cast<int>();

  const auto res = rrtSearch(0, space, 2000, initial, goal);

  // Convert the Path to something the ROS side wants
  std::vector<std::array<float, 2>> ret;
  if (res.has_value()) {
    ret.reserve(res.value().size());
    std::ranges::transform(res.value(), std::back_inserter(ret),
                           [](const auto p) -> std::array<float, 2> {
                             return {static_cast<float>(p.second[0]),
                                     static_cast<float>(p.second[1])};
                           });
  }
  return ret;
}

// Instantiate this template as used in main.cpp
template std::vector<std::array<float, 2>>
rrt(int, int, std::vector<signed char>::const_iterator,
    std::vector<signed char>::const_iterator, std::array<float, 2>,
    std::array<float, 2>);
