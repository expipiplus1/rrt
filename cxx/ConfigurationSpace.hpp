#pragma once

#include "Tree.hpp"
#include <concepts>

// This concept constrains configuration spaces
template <typename I,
          typename Tree_ = Tree<typename I::Reachability, typename I::Config>>
concept ConfigurationSpace = requires(I cs, typename I::Config c, Tree_ t,
                                      typename Tree_::VertexIndex i) {
  // The type of proofs of reachability between configurations. For
  // kinetodynamic configuration spaces this could be the control input for
  // example.
  typename I::Reachability;

  // A type capable of representing each point in a configuration space, for a
  // simple 2-space this could be a 2-vector.
  typename I::Config;

  // Some metric representing distance between points, does not have to be
  // symmetric. nearestNeighbor is defined here too to permit utilising
  // acceleration structures for NN search.
  { cs.distance(c, c) }
  ->std::same_as<double>;
  { cs.nearestNeighbor(t, c) }
  ->std::same_as<std::pair<const typename Tree_::VertexIndex, double>>;

  // Sample a point in the configuration space, the target is passed as a
  // parameter to permit biasing sampling towards that point.
  { cs.sample(c) }
  ->std::same_as<typename I::Config>;

  // Tries to move from the initial configuration towards a target
  // configuration. It returns none or several progress making configurations
  // along with their proofs of reachability.
  // It is also given the tree of all currently reachable configurations.
  { cs.step(t, i, c) }
  ->std::same_as<void>;
};

// A default implementation for `nearestNeighbor` performing a linear scan over
// all the vertices.
template <ConfigurationSpace CS,
          typename Tree_ = Tree<typename CS::Reachability, typename CS::Config>>
std::pair<const typename Tree_::VertexIndex, double>
defaultNearestNeighbor(const CS &cs, const Tree_ &tree,
                       const typename CS::Config c) {
  double best = cs.distance(tree.root, c);
  typename Tree_::VertexIndex bestConfig = Tree_::rootVertex;

  for (std::size_t i = 0; i < tree.size(); ++i) {
    const auto n = tree[i];
    const double nDist = cs.distance(n.config, c);
    if (nDist < best) {
      best = nDist;
      bestConfig = i;
    }
  }

  return std::make_pair(bestConfig, best);
};
