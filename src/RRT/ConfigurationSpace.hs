{-# LANGUAGE RankNTypes #-}

module RRT.ConfigurationSpace
  ( ConfigurationSpace(..)
  , Reachable(..)
  , Direction(..)
  , blossom
  , biasTowardsConfiguration
  , defaultNearestNeighbor
  , twoSpace
  ) where

import           Data.Foldable
import           Data.Ord
import           System.Random

-- | A configuration space parameterized over the randomness source for
-- sampling, @g@, reachability proof @u@, and the type of the configurations,
-- @c@.
data ConfigurationSpace g u c = ConfigurationSpace
  {
  -- | Some metric representing distance between points, does not have to be
  -- symmetric
    distance        :: c -> c -> Double
  , nearestNeighbor :: forall b . [(c, b)] -> c -> (c, Double, b)

  -- | Sample a point in the configuration space, the target is passed as a
  -- parameter to permit biasing sampling towards that point.
  , sample          :: g -> c -> (c, g)

  -- | Tries to move from the initial configuration towards a target
  -- configuration. It returns none or several progress making configurations
  -- along with their proofs of reachability.
  -- It is also given a list of all currently reachable configurations.
  , step            :: Direction -> [c] -> c -> c -> [(u, c)]
  }

data Reachable a = NotReachable | Reachable a

-- | For spaces where movement between configurations is not necessarily
-- reversible this specified whether we are stepping forwards or backwards.
data Direction = Forwards | Backwards

-- | Modify a 'ConfigurationSpace' such that some proportion of the time,
-- sampling the space returns the target configuration. This is typically
-- used to bias the search towards the goal configuration.
--
-- According to https://www.cs.cmu.edu/~motionplanning/lecture/lec20.pdf, 0.05
-- to 0.1 is a good choice, although this isn't qualified further.
biasTowardsConfiguration
  :: RandomGen g
  => Double
  -- ^ The proportion of calls to 'sample' which return the specified configuration
  -> ConfigurationSpace g u c
  -> ConfigurationSpace g u c
biasTowardsConfiguration p conf =
  let sample' g target =
        let (x, g') = randomR (0, 1) g
        in  if x <= p then (target, g') else sample conf g' target
  in  conf { sample = sample' }

-- | Perform a linear search using the specified metric
defaultNearestNeighbor :: (c -> c -> Double) -> [(c, b)] -> c -> (c, Double, b)
defaultNearestNeighbor dist xs y = case xs of
  [] -> error "defaultNearestNeighbor given no neighbors"
  _ ->
    let distances = (\(x, b) -> (x, dist x y, b)) <$> xs
    in  minimumBy (comparing (\(_, d, _) -> d)) distances

----------------------------------------------------------------
-- RRT-Blossom
----------------------------------------------------------------

-- | A step function exploring all continuations of a configuration and
-- ignoring those which are more easily reachable by an existing node in the
-- tree.
blossom
  :: (c -> c -> Double)
  -- ^ Distance metric
  -> (Direction -> c -> c -> [(u, c)])
  -- ^ All possible continuations of a configuration, optionally reaching
  -- towards a target. Must be a finite list.
  -> (Direction -> [c] -> c -> c -> [(u, c)])
  -- ^ The step function
blossom dist simulations direction explored initial target =
  let distance' = case direction of
        Forwards  -> dist
        Backwards -> flip dist
      regression new =
        any (\e -> distance' e new < distance' initial new) explored
      sims = simulations direction initial target
  in  filter (not . regression . snd) sims

----------------------------------------------------------------
-- Some spaces
----------------------------------------------------------------

twoSpace
  :: RandomGen g
  => Double
  -- ^ δ, The proportion of distance to to a new vertex from its nearest neighbor
  -> Double
  -- ^ τ, If the distance to a new vertex is less than this threshold, jump
  -- straight to the new vertex
  -> ConfigurationSpace g () (Double, Double)
twoSpace delta nearThreshold =
  let sq x = x * x
      quadrance (a, b) (c, d) = sq (c - a) + sq (d - b)
      nearThreshold2 = sq nearThreshold
  in  ConfigurationSpace
        { distance        = quadrance
        , nearestNeighbor = defaultNearestNeighbor quadrance
        , sample          = \g _ ->
                              let (x, g' ) = randomR (0, 1) g
                                  (y, g'') = randomR (0, 1) g'
                              in  ((x, y), g'')
        , step            = \_ _ x y -> if quadrance x y <= nearThreshold2
                              then [((), y)]
                              else [((), lerp2 delta x y)]
        }

lerp2 :: Double -> (Double, Double) -> (Double, Double) -> (Double, Double)
lerp2 w (a, b) (c, d) = (a * (1 - w) + c * w, b * (1 - w) + d * w)
