{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE RankNTypes #-}

module RRT
  ( rrtSearch
  , rrtSearchDualTree
  ) where

import           System.Random                  ( RandomGen )

import           Data.Foldable                  ( find
                                                , minimumBy
                                                )
import           Data.Ord                       ( comparing )
import           RRT.ConfigurationSpace

-- | Search for a path from the first configuration to the second.
rrtSearch
  :: forall g u c
   . (RandomGen g, Eq c)
  => g
  -- ^ The randomness source
  -> ConfigurationSpace g u c
  -- ^ The parameters of the configuration space
  -> Word
  -- ^ Maximum number of vertices to search
  -> c
  -- ^ Initial configuration
  -> c
  -- ^ Goal configuration
  -> Maybe (Path u c)
  -- ^ Intermediate configurations from the initial to the goal
rrtSearch initR conf@ConfigurationSpace {..} k start goal =
  let -- 'go' takes a list of paths from configurations to the start
      -- configuration. It:
      --
      -- - Samples from the configuration space, 'rand'
      -- - Grows the tree towards that point
      -- - If the tree has reached the goal, finish, otherwise continue with
      --   the expanded tree.
      go :: (g, Tree u c) -> Loop (Path u c) (g, Tree u c)
      go (g, ps) =
        let (rand    , g' ) = sample g goal
            (newPaths, ps') = growTree conf ps rand
        in  case find (\p -> pathHead p == goal) newPaths of
              Just p ->
                Finish $ constructSingleTreeOutput (direction ps, p) goal
              Nothing -> Continue (g', ps')
  in  loopBounded go k (initR, Tree Forwards start [])

-- | Search for a path from the first configuration to the second and for a
-- reverse path in the other direction, trying to meet in the middle.
rrtSearchDualTree
  :: forall g u c
   . (Show c, RandomGen g, Eq c)
  => g
  -- ^ The randomness source
  -> ConfigurationSpace g u c
  -- ^ The parameters of the configuration space
  -> Word
  -- ^ Maximum number of iterations, in each iteration at most two trees are
  -- extended. The number of vertices is bounded by two times this many calls
  -- to 'step'
  -> c
  -- ^ Initial configuration
  -> c
  -- ^ Goal configuration
  -> Maybe (Path u c)
  -- ^ Intermediate configurations from the initial to the goal
rrtSearchDualTree initR conf@ConfigurationSpace {..} k start goal =
  let
    -- - Sample a new point
    -- - Extend ps towards that point
    -- - If we reached the sister tree's root, finish
    -- - If we couldn't make progress, swap ps and qs and continue
    -- - If we made progress, try to extend qs towards the new point
    -- - If they connected, finish, otherwise swap ps and qs and continue
    go :: (g, Tree u c, Tree u c) -> Loop (Path u c) (g, Tree u c, Tree u c)
    go (g, ps, qs) =
      let -- Sample biased towards the root of the sister tree.
        (rand, g') = sample g (root qs)

        -- The distance metric we should use when comparing paths in the ps
        -- tree. If that tree is operating forwards then the target needs to
        -- be the second parameter and vice-versa.
        distanceP  = case direction ps of
          Forwards  -> flip distance rand
          Backwards -> distance rand

        -- Grow the tree towards the sampled point
        (newPathsP, ps') = growTree conf ps rand
        pathToOtherRoot  = find ((root qs ==) . pathHead) newPathsP
      in
        case minimumOnMay (distanceP . pathHead) newPathsP of
          -- If we reached the root of the other tree, finish here
          _ | Just p <- pathToOtherRoot ->
            Finish $ constructSingleTreeOutput (direction ps', p) goal

          -- We didn't make any progress, swap p and q and iterate
          Nothing -> Continue (g', qs, ps') -- swap qs and ps

          -- If we made progress, try to extend the sister tree towards the
          -- best new point, terminating if we find it.
          Just bestNewPathP ->
            let newP = pathHead bestNewPathP
            in
              case growTree conf qs newP of
                (newPathsQ, _)
                  | Just q <- find ((newP ==) . pathHead) newPathsQ
                  -> Finish $ constructDualTreeOutput
                    (direction ps', bestNewPathP)
                    q
                    goal
                (_, qs') -> Continue (g', qs', ps')
  in
    loopBounded go k (initR, Tree Forwards start [], Tree Backwards goal [])

-- | A 'Path' represented as a series of intermediate points along with their
-- reachability proofs from the previous state.
type Path u c = [(u, c)]

pathHead :: Path u c -> c
pathHead = \case
  []         -> error "pathHead: empty path"
  (_, c) : _ -> c

-- | A 'Tree' is a root and a selection of paths, sharing tails, it also tracks
-- whether it's exploring 'Forwards' or 'Backwards'.
data Tree u c = Tree
  { direction :: Direction
  , root      :: c
  , paths     :: [Path u c]
  }

-- Search a tree for the nearest neighbor to a configuration
nearestNeighbor'
  :: ConfigurationSpace g u c -> Tree a c -> c -> (c, Double, [(a, c)])
nearestNeighbor' ConfigurationSpace {..} Tree {..} =
  nearestNeighbor ((root, []) : ((\p -> (snd (head p), p)) <$> paths))

-- | Try to grow a tree towards a point in the configuration space. If the
-- configuration has already been reached then return that path and the
-- unmodified tree.
growTree :: ConfigurationSpace g u c -> Tree u c -> c -> ([Path u c], Tree u c)
  -- ^ Extend the tree towards a configuration, returning the best new path and
  -- the updated tree. The tree may have additional paths inserted.
growTree conf@ConfigurationSpace {..} t c = case nearestNeighbor' conf t c of
  (_, 0, path) -> ([path], t)
  (near, _, path) ->
    let newConfigs = step (direction t) (pathHead <$> paths t) near c
        newPaths   = (: path) <$> newConfigs
        t'         = t { paths = newPaths ++ paths t }
    in  (newPaths, t')

-- | Append the backwards tree to the forward tree properly
constructDualTreeOutput
  :: (Direction, Path u c)
  -- ^ A path and its polarity
  -> Path u c
  -- ^ A path of the opposite polarity
  -> c
  -- ^ The goal
  -> Path u c
  -- ^ A forward path
constructDualTreeOutput (directionP, ps) qs = case directionP of
  Forwards  -> stitchPaths ps qs
  Backwards -> stitchPaths qs ps

constructSingleTreeOutput :: (Direction, Path u c) -> c -> Path u c
constructSingleTreeOutput (directionP, ps) = case directionP of
  Forwards  -> stitchPaths ps []
  Backwards -> stitchPaths [] ps

stitchPaths :: Path u c -> Path u c -> c -> Path u c
stitchPaths f b goal =
  reverse f ++ zip (fst <$> b) (tail ((snd <$> b) ++ [goal]))

----------------------------------------------------------------
-- Utils
----------------------------------------------------------------

data Loop a b = Finish a | Continue b

-- | Iterate until we reach an early termination ('Finish') or we run out of
-- steps ('Continue')
loopBounded :: (a -> Loop b a) -> Word -> a -> Maybe b
loopBounded f =
  let go k i = if k == 0
        then Nothing
        else case f i of
          Finish   r  -> Just r
          Continue i' -> go (pred k) i'
  in  go

minimumOnMay :: Ord b => (a -> b) -> [a] -> Maybe a
minimumOnMay m = \case
  [] -> Nothing
  xs -> Just $ minimumBy (comparing m) xs
