#pragma once

#include <deque>
#include <eigen3/Eigen/Dense>
#include <optional>
#include <random>
#include <string_view>
#include <variant>
#include <vector>

#include "ConfigurationSpace.hpp"
#include "Tree.hpp"

// `stitchPaths` takes two path iterators, one forwards and one backwards as
// well as the goal configuration.
//
// The forwards path will iterate back towards the root of the forwards tree,
// so must be reversed at the front of the eventual path.
//
// The backwards path is in the same direction, with reversed reachability
// proofs, so is appended to the final path with reachability proofs pointing
// towards the vertex parents rather than the vertices themselves.
//
// Pass PathIterator{nullptr, rootVertex} as either path to indicate an empty
// path.
template <typename C, typename R, std::input_iterator FI,
          std::input_iterator BI>
Path<R, C> stitchPaths(FI forwards, BI backwards, const C goal) {
  Path<R, C> ret;
  std::ranges::copy(forwards, PathEndSentinal{}, std::front_inserter(ret));

  while (backwards != PathEndSentinal{}) {
    const auto reachability = (*backwards).first;
    ++backwards;
    const auto config =
        backwards != PathEndSentinal{} ? (*backwards).second : goal;
    ret.push_back({reachability, config});
  }

  return ret;
}

// A helper for calling `stitchPaths` with a single tree (forwards or backwards)
template <typename C, typename R, typename Tree_ = Tree<R, C>>
Path<R, C> constructSingleTreeOutput(const Tree_ &tree,
                                     typename Tree_::VertexIndex i,
                                     const C goal) {
  if (tree.direction == Forwards) {
    return stitchPaths<C, R>(
        tree.path(i), typename Tree_::PathIterator{nullptr, Tree_::rootVertex},
        goal);
  } else {
    return stitchPaths<C, R>(
        typename Tree_::PathIterator{nullptr, Tree_::rootVertex}, tree.path(i),
        goal);
  }
}

// A helper for calling `stitchPaths` with a two trees (one forwards and one
// backwards). The order of arguments doesn't matter
template <typename C, typename R, typename Tree_ = Tree<R, C>>
Path<R, C>
constructDualTreeOutput(const Tree_ &treeX, typename Tree_::VertexIndex iX,
                        const Tree_ &treeY, typename Tree_::VertexIndex iY,
                        const C goal) {
  if (treeX.direction == Forwards) {
    return stitchPaths<C, R>(treeX.path(iX), treeY.path(iY), goal);
  } else {
    return stitchPaths<C, R>(treeY.path(iY), treeX.path(iX), goal);
  }
}

// Perform a standard RRT search. Returns Nothing if a path could not be found.
template <ConfigurationSpace CS,
          typename Tree_ = Tree<typename CS::Reachability, typename CS::Config>>
std::optional<Path<typename CS::Reachability, typename CS::Config>>
rrtSearch(const int seed, CS &configurationSpace, const unsigned maxIterations,
          const typename CS::Config initialConfig,
          const typename CS::Config goalConfig) {

  // Initialize our empty tree
  Tree_ tree(Forwards, initialConfig);

  // Bound work by maxIterations
  for (unsigned i = 0; i < maxIterations; ++i) {
    // Sample a new point in the configuration space
    const auto rand = configurationSpace.sample(goalConfig);

    // Remember the old tree size so we can identify new vertices
    const auto oldTreeSize = tree.size();

    // Grow the tree towards our new configuration, no need to check the result
    // for this simple search strategy, we'll check it straight after.
    growTree(configurationSpace, tree, rand);

    // For all the new vertices, check if we have found our goal
    for (typename Tree_::VertexIndex j = oldTreeSize; j < tree.size(); ++j) {
      const auto config = tree.config(j);
      if (config == goalConfig) {

        // If we have found our goal, return the reified path.
        return {constructSingleTreeOutput<typename CS::Config,
                                          typename CS::Reachability>(
            tree, j, goalConfig)};
      }
    }
  }
  return {};
}

// Perform a RRT search using two trees, one expanding from the target, one
// expanding from the initial point. Returns Nothing if a path could not be
// found.
template <ConfigurationSpace CS,
          typename Tree_ = Tree<typename CS::Reachability, typename CS::Config>>
std::optional<Path<typename CS::Reachability, typename CS::Config>>
rrtSearchDualTree(
    const int seed, CS &configurationSpace,
    const unsigned maxIterations, // in each iteration at most two trees are
                                  // extended, the number of vertices is bounded
                                  // by two times this many calls to 'step'
    const typename CS::Config initialConfig,
    const typename CS::Config goalConfig) {

  // Initialize our empty trees
  Tree_ treeX(Forwards, initialConfig);
  Tree_ treeY(Backwards, goalConfig);

  // Bound work by maxIterations
  for (unsigned i = 0; i < maxIterations; ++i) {
    // Sample a new point in the configuration space, targeted towards the root
    // of the sister tree
    const auto rand = configurationSpace.sample(treeY.root);

    // Remember the old tree size so we can identify new vertices
    const auto oldTreeSize = treeX.size();

    // Grow the tree towards the sampled point.
    growTree(configurationSpace, treeX, rand);

    // For all the new vertices, check if we have made it to the root of the
    // other tree, return if we did.
    for (typename Tree_::VertexIndex j = oldTreeSize; j < treeX.size(); ++j) {
      const auto config = treeX.config(j);
      if (config == treeY.root) {
        // If we have found the other root return the reified path.
        return {constructSingleTreeOutput<typename CS::Config,
                                          typename CS::Reachability>(
            treeX, j, goalConfig)};
      }
    }

    // If we made progress, try to extend the sister tree towards the best
    // new point.
    if (oldTreeSize != treeX.size()) {
      // The best new path is the one which got us closest to our random sample
      // For simple RRT searches there will usually only be one new point
      //
      // Using the fancy c++20 ranges with projection!
      const auto bestNewPath = std::ranges::min_element(
          treeX.begin() + oldTreeSize, treeX.end(), {}, [&](const auto a) {
            // Because the distance metric is not necessarily symmetric, flip
            // the order of arguments to distance for the reverse tree. (I
            // wonder if the compiler can float out this branch...
            if (treeX.direction == Forwards) {
              return configurationSpace.distance(a.config, rand);
            } else {
              return configurationSpace.distance(rand, a.config);
            }
          });
      const auto oldTreeYSize = treeY.size();
      const auto found =
          growTree(configurationSpace, treeY, bestNewPath->config);

      // If we had already discovered this point, stitch the paths and return
      if (found.has_value()) {
        return {constructDualTreeOutput<typename CS::Config,
                                        typename CS::Reachability>(
            treeX, bestNewPath - treeX.begin(), treeY, found.value(),
            goalConfig)};
      }

      // If we extended treeY, check for any matchups
      for (typename Tree_::VertexIndex j = oldTreeYSize; j < treeY.size();
           ++j) {
        const auto config = treeY.config(j);
        if (config == bestNewPath->config) {
          return {constructDualTreeOutput<typename CS::Config,
                                          typename CS::Reachability>(
              treeX, bestNewPath - treeX.begin(), treeY, j, goalConfig)};
        }
      }
    }

    // If we haven't terminated by this point, swap the trees for the next
    // iteration.
    std::swap(treeX, treeY);
  }
  return {};
}

// Try to grow a tree towards a point in the configuration space, if that point
// has already been reached, return the iterator to that element in the tree
template <ConfigurationSpace CS,
          typename Tree_ = Tree<typename CS::Reachability, typename CS::Config>>
std::optional<const typename Tree_::VertexIndex>
growTree(CS &configurationSpace, Tree_ &tree, const typename CS::Config c) {
  const auto [path, dist] = configurationSpace.nearestNeighbor(tree, c);

  // If we've found a path to this goal, return it
  if (dist == 0) {
    return path;
  }

  // Otherwise, step and insert the results into the tree and return nothing
  configurationSpace.step(tree, path, c);
  return {};
}

// For LSP
#include "Euclidian.hpp"
#include "SpaceModifiers.hpp"
template std::optional<Path<TwoSpace::Reachability, TwoSpace::Config>>
rrtSearch<BiasTowardsGoal<TwoSpace>>(const int, BiasTowardsGoal<TwoSpace>&,
                                     const unsigned, const TwoSpace::Config,
                                     const TwoSpace::Config);
template std::optional<Path<TwoSpace::Reachability, TwoSpace::Config>>
rrtSearchDualTree<BiasTowardsGoal<TwoSpace>>(const int,
                                             BiasTowardsGoal<TwoSpace>&,
                                             const unsigned,
                                             const TwoSpace::Config,
                                             const TwoSpace::Config);
