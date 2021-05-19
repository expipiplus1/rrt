#pragma once

#include <random>
#include "ConfigurationSpace.hpp"

// This class template will sample the goal configuration some proportion of
// the time, otherwise the underlying class's sample implementation is used.
template <ConfigurationSpace CS> class BiasTowardsGoal : private CS {
public:
  // p in [0..1]
  template <typename... Args>
  BiasTowardsGoal(const float p, uint32_t seed, Args &&...args)
      : CS(std::forward<Args>(args)...), p(p), gen(seed) {}

  using CS::distance;
  using CS::nearestNeighbor;
  using CS::step;
  using typename CS::Config;
  using typename CS::Reachability;

  typename CS::Config sample(const typename CS::Config c) {
    if (dis(gen) < p)
      return c;
    return CS::sample(c);
  }

private:
  float p;
  std::mt19937 gen;
  std::uniform_real_distribution<float> dis;
};
