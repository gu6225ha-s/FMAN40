#ifndef IMAGE_H_
#define IMAGE_H_

#include <Eigen/Dense>
#include <cstddef>
#include <string>

namespace ppr {

/// \brief Single channel 8-bit image.
///
/// The center of the upper left pixel has coordinates (0,0). Data is stored in
/// row-major order.
class Image {
public:
  /// \brief Create new, empty image.
  Image() : width_(0), height_(0), data_(nullptr) {}

  /// \brief Create new image of specific size.
  /// \param width image width
  /// \param height image height
  Image(size_t width, size_t height) : width_(width), height_(height) {
    data_ = new unsigned char[width * height];
  }

  /// \brief Deleted copy constructor.
  Image(const Image &) = delete;

  /// \brief Move constructor.
  ///
  /// The new image takes ownership of the pixel data in \p other.
  /// \param other another image
  Image(Image &&other) { *this = std::move(other); }

  /// \brief Destructor.
  ~Image() { delete[] data_; }

  /// \brief Move assignment operator.
  ///
  /// This image takes ownership of the pixel data in \p other.
  /// \param other another image
  /// \return A reference to this image.
  Image &operator=(Image &&other);

  /// \brief Get a pointer to a row of the image.
  /// \param y row index
  /// \return A row pointer.
  unsigned char *operator[](int y) const { return &data_[y * width_]; }

  /// \brief Get the width of the image.
  /// \return Width of the image.
  size_t Width() const { return width_; }

  /// \brief Get the height of the image.
  /// \return Height of the image.
  size_t Height() const { return height_; }

  /// \brief Reallocate the image.
  /// \param width new width of the image
  /// \param height new height of the image
  void Realloc(size_t width, size_t height);

  /// \brief Bilinear interpolation.
  /// \param x x coordinate
  /// \param y y coordinate
  /// \return The interpolated pixel value at (x,y).
  float Interp(float x, float y) const;

  /// \brief Warp into another image using a homography.
  /// \param im image to warp into
  /// \param H homography from \p im to this image
  /// \param offset offset of \p im relative the origin
  void Warp(const Image &im, const Eigen::Matrix3d &H,
            const Eigen::Vector2i &offset) const;

private:
  size_t width_, height_; // Image size
  unsigned char *data_;   // Pixel data
};

/// \brief 8-bit RGB image.
class RgbImage {
public:
  /// \brief Create new, empty image.
  RgbImage() {}

  /// \brief Create new image of specific size.
  /// \param width image width
  /// \param height image height
  RgbImage(size_t width, size_t height)
      : r_(width, height), g_(width, height), b_(width, height) {}

  /// \brief Deleted copy constructor.
  RgbImage(const RgbImage &) = delete;

  /// \brief Move constructor.
  ///
  /// The new image takes ownership of the pixel data in \p other.
  /// \param other another image
  RgbImage(RgbImage &&other) { *this = std::move(other); }

  /// \brief Move assignment operator.
  ///
  /// This image takes ownership of the pixel data in \p other.
  /// \param other another image
  /// \return A reference to this image.
  RgbImage &operator=(RgbImage &&other) {
    r_ = std::move(other.r_);
    g_ = std::move(other.g_);
    b_ = std::move(other.b_);
    return *this;
  }

  /// \brief Get the red image channel.
  /// \return The red image hannel.
  const Image &R() const { return r_; }

  /// \brief Get the green image channel.
  /// \return The green image hannel.
  const Image &G() const { return g_; }

  /// \brief Get the blue image channel.
  /// \return The blue image hannel.
  const Image &B() const { return b_; }

  /// \brief Get the width of the image.
  /// \return Width of the image.
  size_t Width() const { return r_.Width(); }

  /// \brief Get the height of the image.
  /// \return Height of the image.
  size_t Height() const { return r_.Height(); }

  /// \brief Warp into another image using a homography.
  /// \param im image to warp into
  /// \param H homography from \p im to this image
  /// \param offset offset of \p im relative the origin
  void Warp(const RgbImage &im, const Eigen::Matrix3d &H,
            const Eigen::Vector2i &offset) const;

  /// \brief Read JPEG image.
  /// \param path path to the JPEG image
  void ReadJpeg(const std::string &path);

  /// \brief Write JPEG image.
  /// \param path path to the JPEG image
  /// \param quality quality of image (0-100)
  void WriteJpeg(const std::string &path, int quality) const;

private:
  Image r_, g_, b_; // Image channels
};

} // namespace ppr

#endif /* IMAGE_H_ */
