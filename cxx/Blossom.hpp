#pragma once

#include <algorithm>
#include <functional>
#include <random>

#include "ConfigurationSpace.hpp"

//
// This class template will use a step function which implements Blossom-RRT
//
// It is parameterized over the underlying ConfigurationSpace and the type of
// Iterator for the finite set of path extensions required in the `step`
// function.
//
// This inherits publically from the base class without a virtual destructor,
// don't delete it via a reference to the parent class, OK? (or downcast in
// general)
//
template <ConfigurationSpace CS, std::ranges::input_range ContRange>
class Blossom : public CS {
public:
  //
  // Forwards constructor arguments to the parent class
  //
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

  //
  // The crux of this function is that it will add every point suggested by the
  // provided continuation function, as long as they're not better reached by
  // an existing pointk
  //
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
  // The function to get a set of possible path continuations can return any
  // range iterator.
  std::function<ContRange(const Blossom<CS, ContRange> &, Direction,
                          typename CS::Config, typename CS::Config)>
      simulations;
};
