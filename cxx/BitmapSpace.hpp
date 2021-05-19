#pragma once

#include <eigen3/Eigen/Dense>
#include <iterator>
#include <memory>
#include <random>
#include <stb/stb_image.h>
#include <stb/stb_image_write.h>
#include <variant>

#include "ConfigurationSpace.hpp"
#include "Tree.hpp"

//
// Using the Bresenham algorithm, perform `perPixel` at every pixel between two
// configurations. `perPixel` can command the function to stop early by
// returning true;
//
// This function paints a contiguous line, with no diagonal movements.
//
inline Eigen::Vector2i
bresenham4(const Eigen::Vector2i p0, const Eigen::Vector2i p1,
           const std::function<bool(Eigen::Vector2i)> &perPixel) {
  int x0 = p0[0];
  int y0 = p0[1];
  const int x1 = p1[0];
  const int y1 = p1[1];
  const int dx = std::abs(x1 - x0);
  const int dy = std::abs(y1 - y0);
  const int signX = x0 < x1 ? 1 : -1;
  const int signY = y0 < y1 ? 1 : -1;
  int e = 0;
  Eigen::Vector2i ret = p0;
  for (int i = 0; i < dx + dy; i++) {
    int e1 = e + dy;
    int e2 = e - dx;
    if (std::abs(e1) < std::abs(e2)) {
      x0 += signX;
      e = e1;
    } else {
      y0 += signY;
      e = e2;
    }
    if (perPixel({x0, y0})) {
      break;
    }
    ret = {x0, y0};
  }
  return ret;
}

//
// A space which only samples and traverses pixels which have value 0xff.
//
// TODO: should probably just be a wrapper around `TwoSpace`.
//
class BitmapSpace {
public:
  //
  // It will never sample points in untraversable areas, maxSampleAttempts
  // prevents unbounded loops when there are no traversable pixels.
  //
  BitmapSpace(const uint32_t seed, const uint32_t maxSampleAttempts)
      : maxSampleAttempts(maxSampleAttempts) {}
  BitmapSpace() = delete;

  // Integral vectors
  using Config = Eigen::Vector2i;
  // The straight line path between points, needs no elaboration
  using Reachability = std::monostate;

  //
  // Initialize using an image in a file.
  //
  void load(const std::string &filename) {
    const int desiredComponents = 1;
    int actualComponents;
    // Load the image and copy it, it's certainly possible to organize this
    // without a copy (indeed that's how it was originally written) but it's a
    // bit neater this way when it has to coexist with `loadFromMemory`
    const auto data =
        std::unique_ptr<unsigned char, decltype(&stbi_image_free)>(
            stbi_load(filename.c_str(), &width, &height, &actualComponents,
                      desiredComponents),
            stbi_image_free);
    if (!data) {
      throw std::runtime_error("unable to load " + filename);
    };
    imageData.reserve(width * height * actualComponents);
    std::ranges::copy(data.get(),
                      data.get() + width * height * actualComponents,
                      std::back_inserter(imageData));
    numComponents = desiredComponents;
  }

  //
  // Initialize from some data under an input_iterator I
  //
  // The iterators must be exactly width*height elements apart
  //
  template <std::input_iterator I, std::sentinel_for<I> S>
  void loadFromMemory(const int width, const int height, I data, S dataEnd) {
    imageData.reserve(width * height);
    std::ranges::copy(data, dataEnd, std::back_inserter(imageData));
    assert(imageData.size() == width * height &&
           "image data iterator wrong size");
    this->width = width;
    this->height = height;
    this->numComponents = 1;
  }

  //
  // Save our image, along with a red path. For debugging or visualization.
  //
  void saveWithPath(const std::string &filename, const Config init,
                    const Path<Reachability, Config> &path) const {
    const auto ourComponents = 3;
    std::vector<unsigned char> image(height * width * ourComponents);
    const auto index = [&](const int x, const int y,
                           const int c) -> unsigned char * {
      return &image[(y * width + x) * ourComponents + c];
    };
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        *index(x, y, 0) = pixel(x, y);
        *index(x, y, 1) = pixel(x, y);
        *index(x, y, 2) = pixel(x, y);
      }
    }
    Config a = init;
    for (const auto &[_, b] : path) {
      bresenham4(a, b, [&](const Config c) {
        *index(c[0], c[1], 0) = 0xff;
        *index(c[0], c[1], 1) = 0x00;
        *index(c[0], c[1], 2) = 0x00;
        return false;
      });
      a = b;
    }
    stbi_write_png(filename.c_str(), width, height, ourComponents,
                   reinterpret_cast<const void *>(image.data()),
                   width * ourComponents);
  };

private:
  // Synonym to reduce some verbosity
  using Tree_ = Tree<Reachability, Config>;

public:
  double distance(const Config c1, const Config c2) const {
    // If there is no path between the points, they are not reachable
    if (rayCast(c1, c2) != c2) {
      return std::numeric_limits<float>::infinity();
    }
    return (c1 - c2).squaredNorm();
  };

  std::pair<const typename Tree_::VertexIndex, double>
  nearestNeighbor(const Tree_ &tree, const Config c) const {
    return defaultNearestNeighbor(*this, tree, c);
  };

  //
  // Never sample an untraversable point
  //
  Config sample(const Config c) {
    for (uint32_t i = 0; i < maxSampleAttempts; ++i) {
      const auto coord =
          Config(std::uniform_int_distribution<int>(0, width - 1)(gen),
                 std::uniform_int_distribution<int>(0, height - 1)(gen));
      if (!occupied(coord)) {
        return coord;
      }
    }
    throw std::runtime_error("unable to find unoccupied pixel");
  }

  //
  // Cast a ray to a point and move half way towards the nearest obstacle, or
  // all the way towards an unobstructed target
  //
  void step(Tree_ &tree, typename Tree_::VertexIndex initial,
            const Config targetConfig) {
    const auto initialConfig = tree.config(initial);
    const auto nearestHit = rayCast(initialConfig, targetConfig);
    const auto halfWay =
        (nearestHit + initialConfig).cwiseQuotient(Config::Constant(2));
    tree.push_back({initial, {}, halfWay});
    if (nearestHit == targetConfig) {
      tree.push_back({initial, {}, targetConfig});
    } else if (halfWay != initialConfig) {
      tree.push_back({initial, {}, halfWay});
    }
  }

  //
  // Some utilities
  //
  int getWidth() const { return width; }
  int getHeight() const { return height; }
  Config rayCast(const Config p0, const Config p1) const {
    const auto max = Config{getWidth() - 1, getHeight() - 1};
    const auto min = Config{0, 0};
    const auto p1Clamped = p1.cwiseMin(max).cwiseMax(min);
    return bresenham4(p0, p1Clamped,
                      [&](const Config p) { return occupied(p); });
  };

private:
  std::vector<unsigned char> imageData;
  int width;
  int height;
  int numComponents;

  uint32_t maxSampleAttempts;

  std::mt19937 gen;

  //
  // Indexing into the image
  //
  unsigned char pixel(const int x, const int y) const {
    const int component = 0;
    assert(x < width && "x out of bounds");
    assert(y < height && "y out of bounds");
    return imageData[(y * width + x) * numComponents + component];
  }
  unsigned char occupied(const Config c) const {
    return pixel(c[0], c[1]) != 0xff;
  }
};

// Check we satisfy the ConfigurationSpace concept
static_assert(ConfigurationSpace<BitmapSpace>);
