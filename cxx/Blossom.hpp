#pragma once

#include <algorithm>
#include <functional>
#include <random>

#include "ConfigurationSpace.hpp"

// This class template will use a step function which implements Blossom-RRT
template <ConfigurationSpace CS, std::ranges::input_range ContRange>
class Blossom : public CS {
public:
  // p in [0..1]
  template <typename... Args>
  Blossom(std::function<ContRange(const Blossom<CS, ContRange> &, Direction,
                                  typename CS::Config, typename CS::Config)>
              simulations,
          Args &&...args)
      : CS(std::forward<Args>(args)...), simulations(std::move(simulations)) {}

  using CS::distance;
  using CS::nearestNeighbor;
  using CS::sample;
  using typename CS::Config;
  using typename CS::Reachability;

  void step(Tree<Reachability, Config> &tree,
            typename Tree<Reachability, Config>::VertexIndex initialIndex,
            Config target) {
    const auto initial = tree.config(initialIndex);

    // Get a list of every continuation from this state
    const auto sims = simulations(*this, tree.direction, initial, target);

    // A continuation is a regression if any existing point is closer to the
    // new config
    const auto regression = [&](const auto newConfig) {
      // GCC has an ICE trying to pass this implicitly!
      const auto &this_(*this);
      return std::ranges::any_of(tree, [&](const auto e) {
        if (tree.direction == Forwards) {
          return this_.distance(e.config, newConfig) <
                 distance(initial, newConfig);
        } else {
          return this_.distance(newConfig, e.config) <
                 distance(newConfig, initial);
        }
      });
    };
    for (const auto sim : sims) {
      if (regression(sim.second)) {
        continue;
      }
      tree.push_back(typename Tree<Reachability, Config>::Vertex(
          initialIndex, sim.first, sim.second));
    }
  };

private:
  std::function<ContRange(const Blossom<CS, ContRange>&, Direction,
                          typename CS::Config, typename CS::Config)>
      simulations;
};
