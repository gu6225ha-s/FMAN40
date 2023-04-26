#ifndef IMAGE_H_
#define IMAGE_H_

#include <cstddef>

namespace ppr {

class Image {
public:
  Image(size_t width, size_t height) : width_(width), height_(height) {
    data_ = new unsigned char[width * height];
  }
  ~Image() { delete[] data_; }
  size_t Width() { return width_; }
  size_t Height() { return height_; }
  unsigned char *operator[](int y) const { return &data_[y * width_]; }
  float Interp(float x, float y);

private:
  size_t width_, height_; // Image size
  unsigned char *data_;   // Pixel data
};

} // namespace ppr

#endif /* IMAGE_H_ */
