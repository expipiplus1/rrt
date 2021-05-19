#pragma once

#include <cstddef>
#include <deque>
#include <utility>
#include <variant>
#include <vector>

// A Path is a list of configurations along with the proof of reachability from
// the prior configuration.
template <typename R, typename C> using Path = std::deque<std::pair<R, C>>;

// For bidirectional RRT, we need to track if the tree is expanding from the
// goal or the initial configuration.
enum Direction { Forwards, Backwards };

// A Sentinal for tree PathIterators
class PathEndSentinal {};

// A Tree is a collection of paths, it is stored as a list of vertices along
// with their parent index and proof of reachability `R` from the parent if the
// direction is `Forwards` or the proof *to* the parent is the direction is
// `Backwards`.
//
// Supports amortized O(1) insertion and O(n) path extraction (nothing
// surprising here).
template <typename R, typename C> class Tree {
public:
  Tree(Direction direction, C root)
      : direction(direction), root(std::move(root)){};

  using VertexIndex = std::size_t;
  // A distinguished `VertexIndex` indicating that the parent is the tree root.
  const static VertexIndex rootVertex = ~0;

  // `Vertex`s store a pointer to their parent in this tree, their lifetime
  // must be bounded by their Tree object.
  struct Vertex {
    VertexIndex parent; // rootVertex to indicate that the parent is the root
    R reachability; // The proof of reachability from the parent to this config
    C config;       // This vertex configuration
  };
  using Vertices = std::vector<Vertex>;

  //
  // Our members
  //
  Direction direction;
  C root;
  Vertices vertices;

  // This iterator permits traversing a tree from a vertex towards the root.
  // Stop at the root using PathEndSentinal. Nothing interesting here, just
  // boilerplate.
  class PathIterator {
  public:
    // Iterators must be default constructable, not too useful
    PathIterator() : tree(nullptr), i(rootVertex) {}
    PathIterator(const Tree *tree, const VertexIndex i) : tree(tree), i(i) {}
    // Horrible that input_iterators have to have difference_type defined and
    // never used, what does it even mean for this linked-list iterator.
    using difference_type = std::ptrdiff_t;
    using value_type = std::pair<R, C>;

    value_type operator*() const {
      const auto edge = (*tree)[i];
      return {edge.reachability, edge.config};
    }
    value_type *operator->() { return &(*tree)[i]; }
    PathIterator &operator++() {
      i = (*tree)[i].parent;
      return *this;
    }
    PathIterator operator++(int) {
      auto ret = *this;
      i = (*tree)[i].parent;
      return ret;
    }
    friend bool operator==(const PathIterator &a, const PathIterator &b) {
      return a.i == b.i;
    };
    friend bool operator==(const PathIterator &a, const PathEndSentinal &) {
      return a.i == rootVertex;
    };
    friend bool operator!=(const PathIterator &a, const PathIterator &b) {
      return !(a == b);
    };
    friend bool operator!=(const PathIterator &a, const PathEndSentinal &b) {
      return !(a == b);
    };

  private:
    const Tree *tree;
    VertexIndex i;
  };
  static_assert(std::input_iterator<PathIterator>);

  //
  // Some standard operations:
  //

  // Iterators to walk through all the vertices
  typename Vertices::const_iterator begin() const { return vertices.begin(); }
  typename Vertices::const_iterator end() const { return vertices.end(); }
  using const_iterator = typename Vertices::const_iterator;

  // Insertion
  void push_back(Vertex v) { vertices.push_back(v); }

  // Querying
  std::size_t size() const { return vertices.size(); }

  // Index vertices, no bounds checking YOLO
  const Vertex &operator[](std::size_t i) const { return vertices[i]; }
  Vertex &operator[](std::size_t i) { return vertices[i]; }

  // Get a configuration for a vertex, correcly handles getting the vertex for
  // the root node.
  const C config(VertexIndex i) {
    if (i == rootVertex) {
      return root;
    } else {
      return vertices[i].config;
    }
  }

  // Get the iterator for a path from a vertex to the root
  PathIterator path(const VertexIndex i) const {
    return PathIterator{this, i};
  };
};

// Some sanity checking:

// Really we want to universally quantify the Tree here,
// i.e. say ∀ r c.  sentinel_for<PathEndSentinal, Tree<r,c>>;
//
// That's a bit much for c++'s anemic type system thogh.
static_assert(
    std::sentinel_for<PathEndSentinal,
                      Tree<std::monostate, std::monostate>::PathIterator>);
