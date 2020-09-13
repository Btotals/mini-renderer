#ifndef CANVAS_H
#define CANVAS_H

#include "../lib/tga/tga_image.h"
#include "../util/geo.h"

class Canvas {
protected:
  // TGAImage image_;
  bool zbuffer_state_;
  float* zbuffer_;

  int width_;
  int height_;
  TGAFormat format_;

public:
  TGAImage image_;

  Canvas();
  Canvas(int width, int height, TGAFormat format);

  int get_width();
  int get_height();
  TGAFormat get_format();

  void enable_zbuffer();
  void disable_zbuffer();

  void clear();

  void write_tga_file(const char* filename, bool rle = true);

  void scale(int width, int height);

  void flip_horizontally();
  void flip_vertically();

  TGAColor get_pixel_color(int x, int y);
  void set_pixel_color(int x, int y, const TGAColor& color);
  unsigned char* get_buffer();

  // draw methods
  void line(int x1, int y1, int x2, int y2, const TGAColor& color);
  void line(const Vector2i& v1, const Vector2i& v2, const TGAColor& color);

  void triangle(int x1,
                int y1,
                int x2,
                int y2,
                int x3,
                int y3,
                const TGAColor& color);
  void triangle(Vector2i v1, Vector2i v2, Vector2i v3, const TGAColor& color);

  void triangle_barycentric_2d(const Vector2i& v1,
                               const Vector2i& v2,
                               const Vector2i& v3,
                               const TGAColor& color);

  void triangle_barycentric_3d(const Vector3f& v1,
                               const Vector3f& v2,
                               const Vector3f& v3,
                               const TGAColor& color);

  void triangle_gouraud(Vector3i v1,
                        Vector3i v2,
                        Vector3i v3,
                        float intensity0,
                        float intensity1,
                        float intensity2);
};

#endif  // CANVAS_H
