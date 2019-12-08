#include "stdlib.h"
#include <algorithm>

#include "../util/util.h"
#include "canvas.h"

Canvas::Canvas() {
  image_ = TGAImage();
};

Canvas::Canvas(int width, int height, TGAFormat format)
  : image_(width, height, format), width_(width), height_(height),
    format_(format) {
  zbuffer_ = nullptr;
}

int Canvas::get_width() {
  return width_;
}

int Canvas::get_height() {
  return height_;
}

TGAFormat Canvas::get_format() {
  return format_;
}

void Canvas::enable_zbuffer() {
  if (zbuffer_ == nullptr) {
    int size = width_ * height_;
    zbuffer_ = new float[size];
    float negative_maxium = -std::numeric_limits<float>::max();
    for (int i = 0; i < size; i++) {
      zbuffer_[i] = negative_maxium;
    }
  }

  zbuffer_state_ = true;
}

void Canvas::disable_zbuffer() {
  zbuffer_state_ = false;
}

void Canvas::clear() {
  image_.clear();
}

void Canvas::write_tga_file(const char* filename, bool rle) {
  image_.write_tga_file(filename, rle);
}

void Canvas::scale(int width, int height) {
  image_.scale(width, height);
}

void Canvas::flip_horizontally() {
  image_.flip_horizontally();
}

void Canvas::flip_vertically() {
  image_.flip_vertically();
}

TGAColor Canvas::get_pixel_color(int x, int y) {
  return image_.get(x, y);
}

void Canvas::set_pixel_color(int x, int y, const TGAColor& color) {
  image_.set(x, y, color);
}

unsigned char* Canvas::get_buffer() {
  return image_.buffer();
}

void Canvas::line(int x1, int y1, int x2, int y2, const TGAColor& color) {
  // which means slope greater than 1
  bool steep = false;
  if (abs(x1 - x2) < abs(y1 - y2)) {
    steep = true;

    swap(x1, y1);
    swap(x2, y2);
  }

  if (x1 > x2) {
    swap(x1, x2);
    swap(y1, y2);
  }

  int delta_x = x2 - x1;
  int delta_y = y2 - y1;
  int derror2 = abs(delta_y) * 2;
  int error2 = 0;
  int y = y1;

  for (int x = x1; x <= x2; x++) {
    if (steep) {
      image_.set(y, x, color);
    } else {
      image_.set(x, y, color);
    }

    error2 += derror2;
    if (error2 > delta_x) {
      y += (y2 > y1 ? 1 : -1);
      error2 -= delta_x * 2;
    }
  }
}

void Canvas::line(const Vector2i& v1,
                  const Vector2i& v2,
                  const TGAColor& color) {
  line(v1.x, v1.y, v2.x, v2.y, color);
}

// 扫描线算法
void Canvas::triangle(Vector2i v1,
                      Vector2i v2,
                      Vector2i v3,
                      const TGAColor& color) {
  // 手动排序，v123 按照 y 轴大小升序
  if (v1.y > v2.y)
    swap(v1, v2);
  if (v1.y > v3.y)
    swap(v1, v3);
  if (v2.y > v3.y)
    swap(v2, v3);

  // 整个三角形的 delta_y
  int total_height = v3.y - v1.y;

  // 先画上半边
  for (int y = v1.y; y < v2.y; y++) {
    // 上半部分的 delta_y
    int segment_height = v2.y - v1.y + 1;
    float alpha = (float)(y - v1.y) / total_height,
          beta = (float)(y - v1.y) / segment_height;

    Vector2i vec_v1v3 = v1 + (v3 - v1) * alpha,
             vec_v1v2 = v1 + (v2 - v1) * beta;

    if (vec_v1v3.x > vec_v1v2.x) {
      swap(vec_v1v3, vec_v1v2);
    }

    for (int x = vec_v1v3.x; x <= vec_v1v2.x; x++) {
      image_.set(x, y, color);
    }
  }

  // 这里画下半边
  for (int y = v2.y; y <= v3.y; y++) {
    int segment_height = v3.y - v2.y + 1;
    float alpha = (float)(y - v1.y) / total_height,
          beta = (float)(y - v2.y) / segment_height;

    Vector2i vec_v1v3 = v1 + (v3 - v1) * alpha,
             vec_v2v3 = v2 + (v3 - v2) * beta;

    if (vec_v1v3.x > vec_v2v3.x) {
      swap(vec_v1v3, vec_v2v3);
    }

    for (int x = vec_v1v3.x; x <= vec_v2v3.x; x++) {
      image_.set(x, y, color);
    }
  }
}

inline Vector3f barycentric(const Vector3f& u, const Vector3f& v) {
  Vector3f p = u ^ v;
  if (abs(p.z) < 1) {
    return Vector3f(-1, 1, 1);
  }
  return Vector3f(1.f - (p.x + p.y) / p.z, p.y / p.z, p.x / p.z);
}

void Canvas::triangle_barycentric_2d(const Vector2i& v1,
                                     const Vector2i& v2,
                                     const Vector2i& v3,
                                     const TGAColor& color) {
  triangle_barycentric_3d(Vector3f(v1.x, v1.y, 0.f), Vector3f(v2.x, v2.y, 0.f),
                          Vector3f(v3.x, v3.y, 0.f), color);
}

// 包围盒 + 重心坐标算法
void Canvas::triangle_barycentric_3d(const Vector3f& v1,
                                     const Vector3f& v2,
                                     const Vector3f& v3,
                                     const TGAColor& color) {
  // 计算 aabb 包围盒
  int max_x = min(max(v1.x, max(v2.x, v3.x)), static_cast<float>(width_ - 1));
  int min_x = min(v1.x, min(v2.x, v3.x));
  int max_y = min(max(v1.y, max(v2.y, v3.y)), static_cast<float>(height_ - 1));
  int min_y = min(v1.y, min(v2.y, v3.y));

  // printf("min(%d, %d), max(%d, %d)\n", min_x, min_y, max_x, max_y);

  Vector2i p;
  for (p.x = min_x; p.x <= max_x; p.x++) {
    for (p.y = min_y; p.y <= max_y; p.y++) {
      Vector3f u(v2.x - v1.x, v3.x - v1.x, v1.x - p.x),
        v(v2.y - v1.y, v3.y - v1.y, v1.y - p.y);

      Vector3f result = barycentric(u, v);
      // std::cout << result << std::endl;

      if (result.x < 0 || result.y < 0 || result.z < 0) {
        continue;
      }

      // 用重心坐标的三个分量分别乘以三角形三个顶点的 z 分量，求出点 p 对应的 z
      // 深度
      float z = result.x * v1.z + result.y * v2.z + result.z * v3.z;
      int index = p.x + p.y * width_;
      if (zbuffer_state_) {
        if (zbuffer_[index] < z) {
          zbuffer_[index] = z;
          image_.set(p.x, p.y, color);
        }
      } else {
        image_.set(p.x, p.y, color);
      }
    }
  }
}
