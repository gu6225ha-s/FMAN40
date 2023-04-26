#include "image.h"
#include <cassert>
#include <cmath>
#include <csetjmp>
#include <jpeglib.h>

namespace ppr {

void Image::Resize(size_t width, size_t height) {
  delete[] data_;
  width_ = width;
  height_ = height;
  data_ = new unsigned char[width * height];
}

float Image::Interp(float x, float y) const {
  assert(0 <= x && x < width_ - 1 && 0 <= y && y < height_ - 1);
  auto lerp = [](float a, float b, float t) { return a + t * (b - a); };
  int x0 = floor(x), y0 = floor(y);
  float v1 = lerp((*this)[y0][x0], (*this)[y0][x0 + 1], x - x0);
  float v2 = lerp((*this)[y0 + 1][x0], (*this)[y0 + 1][x0 + 1], x - x0);
  return lerp(v1, v2, y - y0);
}

void Image::Warp(const Image &im, const Eigen::Matrix3d &H,
                 const Eigen::Vector2i &offset) const {
  for (int y = 0; y < im.Height(); y++) {
    for (int x = 0; x < im.Width(); x++) {
      Eigen::Vector3d p(x + offset.x(), y + offset.y(), 1);
      p = H * p;
      p /= p.z();
      if (p.x() < 0) {
        p.x() = 0;
      } else if (p.x() >= width_ - 1) {
        p.x() = width_ - 1.001;
      }
      if (p.y() < 0) {
        p.y() = 0;
      } else if (p.y() >= height_ - 1) {
        p.y() = height_ - 1.001;
      }
      im[y][x] = Interp(p.x(), p.y());
    }
  }
}

void RgbImage::Warp(const RgbImage &im, const Eigen::Matrix3d &H,
                    const Eigen::Vector2i &offset) const {
  R().Warp(im.R(), H, offset);
  G().Warp(im.G(), H, offset);
  B().Warp(im.B(), H, offset);
}

struct my_error_mgr {
  struct jpeg_error_mgr pub;
  jmp_buf setjmp_buffer;
};

static void my_error_exit(j_common_ptr cinfo) {
  struct my_error_mgr *myerr = (struct my_error_mgr *)cinfo->err;
  (*cinfo->err->output_message)(cinfo);
  longjmp(myerr->setjmp_buffer, 1);
}

void RgbImage::ReadJpeg(const std::string &path) {
  struct jpeg_decompress_struct cinfo;
  struct my_error_mgr jerr;
  FILE *fp;

  if ((fp = fopen(path.c_str(), "rb")) == nullptr) {
    throw std::runtime_error("Failed to open JPEG file " + path);
  }

  cinfo.err = jpeg_std_error(&jerr.pub);
  jerr.pub.error_exit = my_error_exit;
  if (setjmp(jerr.setjmp_buffer)) {
    jpeg_destroy_decompress(&cinfo);
    fclose(fp);
    throw std::runtime_error("Error reading JPEG file " + path);
  }

  jpeg_create_decompress(&cinfo);
  jpeg_stdio_src(&cinfo, fp);
  cinfo.out_color_space = JCS_RGB;

  jpeg_read_header(&cinfo, TRUE);
  jpeg_start_decompress(&cinfo);

  int row_stride = cinfo.output_width * cinfo.output_components;
  JSAMPARRAY buffer = (*cinfo.mem->alloc_sarray)((j_common_ptr)&cinfo,
                                                 JPOOL_IMAGE, row_stride, 1);

  r_.Resize(cinfo.output_width, cinfo.output_height);
  g_.Resize(cinfo.output_width, cinfo.output_height);
  b_.Resize(cinfo.output_width, cinfo.output_height);

  while (cinfo.output_scanline < cinfo.output_height) {
    unsigned char *r = r_[cinfo.output_scanline],
                  *g = g_[cinfo.output_scanline],
                  *b = b_[cinfo.output_scanline];
    jpeg_read_scanlines(&cinfo, buffer, 1);
    for (int i = 0; i < cinfo.output_width; i++) {
      r[i] = (*buffer)[3 * i + 0];
      g[i] = (*buffer)[3 * i + 1];
      b[i] = (*buffer)[3 * i + 2];
    }
  }

  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
  if (fclose(fp)) {
    throw std::runtime_error("Failed to close JPEG file " + path);
  }
}

void RgbImage::WriteJpeg(const std::string &path, int quality) const {
  struct jpeg_compress_struct cinfo;
  struct my_error_mgr jerr;
  FILE *fp;

  if ((fp = fopen(path.c_str(), "wb")) == nullptr) {
    throw std::runtime_error("Failed to open JPEG file " + path);
  }

  cinfo.err = jpeg_std_error(&jerr.pub);
  jerr.pub.error_exit = my_error_exit;
  if (setjmp(jerr.setjmp_buffer)) {
    jpeg_destroy_compress(&cinfo);
    fclose(fp);
    throw std::runtime_error("Error writing JPEG file " + path);
  }

  jpeg_create_compress(&cinfo);
  jpeg_stdio_dest(&cinfo, fp);
  cinfo.image_width = R().Width();
  cinfo.image_height = R().Height();
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_RGB;
  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE);

  jpeg_start_compress(&cinfo, TRUE);

  int row_stride = R().Width() * 3;
  JSAMPARRAY buffer = (*cinfo.mem->alloc_sarray)((j_common_ptr)&cinfo,
                                                 JPOOL_IMAGE, row_stride, 1);

  while (cinfo.next_scanline < cinfo.image_height) {
    unsigned char *r = r_[cinfo.next_scanline], *g = g_[cinfo.next_scanline],
                  *b = b_[cinfo.next_scanline];
    for (int i = 0; i < cinfo.image_width; i++) {
      (*buffer)[3 * i + 0] = r[i];
      (*buffer)[3 * i + 1] = g[i];
      (*buffer)[3 * i + 2] = b[i];
    }
    jpeg_write_scanlines(&cinfo, buffer, 1);
  }

  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);
  if (fclose(fp)) {
    throw std::runtime_error("Failed to close JPEG file " + path);
  }
}

} // namespace ppr
