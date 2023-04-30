#ifndef IMAGE_H_
#define IMAGE_H_

#include <Eigen/Dense>
#include <cstddef>
#include <string>

namespace ppr {

class Image {
public:
  Image() : width_(0), height_(0), data_(nullptr) {}
  Image(size_t width, size_t height) : width_(width), height_(height) {
    data_ = new unsigned char[width * height];
  }
  Image(const Image &) = delete;
  Image(Image &&other) { *this = std::move(other); }
  ~Image() { delete[] data_; }

  Image &operator=(Image &&other);
  unsigned char *operator[](int y) const { return &data_[y * width_]; }

  size_t Width() const { return width_; }
  size_t Height() const { return height_; }

  void Realloc(size_t width, size_t height);
  float Interp(float x, float y) const;
  void Warp(const Image &im, const Eigen::Matrix3d &H,
            const Eigen::Vector2i &offset) const;

private:
  size_t width_, height_; // Image size
  unsigned char *data_;   // Pixel data
};

class RgbImage {
public:
  RgbImage() {}
  RgbImage(size_t width, size_t height)
      : r_(width, height), g_(width, height), b_(width, height) {}
  RgbImage(const RgbImage &) = delete;
  RgbImage(RgbImage &&other) { *this = std::move(other); }

  RgbImage &operator=(RgbImage &&other) {
    r_ = std::move(other.r_);
    g_ = std::move(other.g_);
    b_ = std::move(other.b_);
    return *this;
  }

  const Image &R() const { return r_; }
  const Image &G() const { return g_; }
  const Image &B() const { return b_; }

  void Warp(const RgbImage &im, const Eigen::Matrix3d &H,
            const Eigen::Vector2i &offset) const;

  void ReadJpeg(const std::string &path);
  void WriteJpeg(const std::string &path, int quality) const;

private:
  Image r_, g_, b_; // Image channels
};

} // namespace ppr

#endif /* IMAGE_H_ */
