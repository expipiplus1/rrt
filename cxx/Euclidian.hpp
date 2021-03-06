#pragma once

#include <eigen3/Eigen/Dense>
#include <random>
#include <variant>

#include "ConfigurationSpace.hpp"
#include "Tree.hpp"

//
// Euclidian spaces parameterized over scalar type and dimensionality.
//
// Points are sampled from [0..1]^D
//
// The naive step function (step straight to the configuration) isn't very
// interesting, so we have a function which: If the target configuration is
// within striking distance, steps straight to it otherwise moves 20% of the
// way towards the target configuration.
//
template <typename T, int D> class Euclidian {
public:
  Euclidian(const uint32_t seed)
      // nonsense max bound here to get the closed interval [0..1]
      : gen(seed), dis(0, std::nextafter(1, std::numeric_limits<T>::max())) {}

  using Config = Eigen::Matrix<T, 1, D>;
  // The straight line path between points doesn't need any explanation
  using Reachability = std::monostate;

private:
  // Synonym to reduce some verbosity
  using Tree_ = Tree<Reachability, Config>;

public:
  // We rather sneakily use the quadrance here insteaf of the L2-norm to save
  // an unnecessary square root operation.
  double distance(const Config c1, const Config c2) const {
    return (c1 - c2).squaredNorm();
  };

  std::pair<const typename Tree_::VertexIndex, double>
  nearestNeighbor(const Tree_ &tree, const Config c) const {
    return defaultNearestNeighbor(*this, tree, c);
  };

  Config sample(const Config c) {
    // This nonsense, not a huge fan of Eigen.
    return Config::NullaryExpr([&]() { return dis(gen); });
  }

  // See comment above
  void step(Tree_ &tree, typename Tree_::VertexIndex initial,
            const Config targetConfig) {
    const auto nearThreshold = 0.1;
    const auto delta = 0.2;
    const auto nearThreshold2 = nearThreshold * nearThreshold;
    const auto initialConfig = tree.config(initial);
    const auto newConfig =
        distance(initialConfig, targetConfig) <= nearThreshold2
            ? targetConfig
            : ((1 - delta) * initialConfig + delta * targetConfig);
    tree.push_back({initial, {}, newConfig});
  }

private:
  std::mt19937 gen;
  std::uniform_real_distribution<float> dis;
};

using TwoSpace = Euclidian<float, 2>;
using ThreeSpace = Euclidian<float, 3>;

// Some sanity checks
static_assert(ConfigurationSpace<TwoSpace>);
static_assert(ConfigurationSpace<ThreeSpace>);
