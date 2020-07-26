#include "./shader.h"
#include <cmath>
#include <cstdlib>
#include <limits>

IShader::~IShader() {}

Matrix44f model_view;
Matrix44f view_port;
Matrix44f projection;

// viewport * projection 用于将整个 [-1, 1] * [-1, 1] * [-1, 1] 的单位空间
// 映射到 [x, x + w] * [y, y + h] * [0, d]
// viewport matrix:
// w/2 0   0   w/2 + x
// 0   h/2 0   h/2 + y
// 0   0   d/2 d/2
// 0   0   0   1

/**
 * @brief 将整个 [-1, 1] * [-1, 1] * [-1, 1] 的单位空间
 * 变换到 [x, x + w] * [y, y + h] * [0, d]
 *
 * @param x 空间坐标 x
 * @param y 空间坐标 y
 * @param w 立方体的宽度
 * @param h 立方体的高度
 * @param depth 立方体的深度
 * @return Matrix viewport 视口变换矩阵
 */
void viewport(int x, int y, int w, int h, int depth) {
  view_port = Matrix44f::identity();
  view_port[0][3] = x + w / 2.f;
  view_port[1][3] = y + h / 2.f;
  view_port[2][3] = depth / 2.f;
  view_port[0][0] = w / 2.f;
  view_port[1][1] = h / 2.f;
  view_port[2][2] = depth / 2.f;

  std::cout << "view_port matrix" << std::endl;
  std::cout << view_port << std::endl;
}

/**
 * @brief 设置透视缩放分量
 *
 * @param coefficient 透视缩放分量
 * @return Matrix projection 投影矩阵
 */
void project(float coefficient) {
  projection = Matrix44f::identity();
  projection[3][2] = coefficient;

  std::cout << "projection matrix" << std::endl;
  std::cout << projection << std::endl;
}

/**
 * @brief 将世界坐标转化为用户视野坐标
 *
 * @param position 相机位置
 * @param center 场景原点
 * @param up 上向量，用于定义相机的正 x 轴
 * @return Matrix 相机/视觉 空间 (Camera/Eye Space)
 */
void lookat(Vector3f eye, Vector3f center, Vector3f up) {
  Vector3f z = (eye - center).normalize(), x = (up ^ z).normalize(),
           y = (z ^ x).normalize();

  model_view = Matrix44f::identity();

  for (int i = 0; i < 3; i++) {
    model_view[0][i] = x[i];
    model_view[1][i] = y[i];
    model_view[2][i] = z[i];
  }

  std::cout << "model_view matrix" << std::endl;
  std::cout << model_view << std::endl;
}

/**
 * @brief 计算点 p 相对于三角形 abc 的重心坐标
 *
 * @param a 三角形点 a
 * @param b 三角形点 b
 * @param c 三角形点 c
 * @param p 待计算点 p
 * @return Vector3f 重心坐标分量
 */
Vector3f barycentric(Vector2f a, Vector2f b, Vector2f c, Vector2f p) {
  Vector3f s[2];

  for (int i = 2; i--;) {
    s[i][0] = c[i] - a[i];
    s[i][1] = b[i] - a[i];
    s[i][2] = a[i] - p[i];
  }

  Vector3f u = s[0] ^ s[1];
  if (abs(u[2]) > 1e-2) {
    return Vector3f(1.f - (u.x + u.y) / u.z, u.y / u.z, u.x / u.z);
  }

  return Vector3f(-1, 1, 1);
}

/**
 * @brief draw a triangle with shader
 *
 * @param pts Vector4f[3] points in world space of triangle
 * @param shader Shader shader with vertex and fragment unit
 * @param img img to be draw
 * @param zbuffer zbuffer img to be draw
 */
void triangle(Vector4f* pts,
              IShader& shader,
              TGAImage& img,
              TGAImage& zbuffer) {
  // calculating aabb boundary box of triangle
  const float float_max = std::numeric_limits<float>::max();
  Vector2f bbox_min(float_max, float_max), bbox_max(-float_max, -float_max);

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      bbox_max[j] = max(bbox_max[j], pts[i][j] / pts[i][3]);
      bbox_min[j] = min(bbox_min[j], pts[i][j] / pts[i][3]);
    }
  }

  Vector2i p;
  TGAColor color;

  // foreach point in bbox, calculating its coordination of barycentric
  for (p.x = bbox_min.x; p.x < bbox_max.x; p.x++) {
    for (p.y = bbox_min.y; p.y < bbox_max.y; p.y++) {
      Vector3f c = barycentric(project<2>(pts[0] / pts[0][3]),
                               project<2>(pts[1] / pts[1][3]),
                               project<2>(pts[2] / pts[2][3]), project<2>(p));

      // calculating current point z-value by adding z-value of 3 pts
      float z = pts[0][2] * c.x + pts[1][2] * c.y + pts[2][2] * c.z;
      // calculating w-value (homogeneous axis)
      float w = pts[0][3] * c.x + pts[1][3] * c.y + pts[2][3] * c.z;
      // mapping to 0 - 255
      int frag_depth = max(0, min(255, int(z / w + .5)));

      // z-buffer test
      if (c.x < 0 || c.y < 0 || c.z < 0 ||
          zbuffer.get(p.x, p.y)[0] > frag_depth) {
        continue;
      }

      bool discard = shader.fragment(c, color);
      if (!discard) {
        zbuffer.set(p.x, p.y, TGAColor(frag_depth));
        img.set(p.x, p.y, color);
      }
    }
  }
}

Vector4f GouraudShader::vertex(int iface, int nthvert) {
  varying_uv.set_column(nthvert, model->uv(iface, nthvert));
  // read the vertex from .obj file
  Vector4f gl_vertex = embed<4>(model->vert(iface, nthvert));

  gl_vertex = view_port * projection * model_view * gl_vertex;

  // get diffuse lighting intensity
  varying_intensity[nthvert] =
    max(0.f, model->norm(iface, nthvert) * light_dir);

  return gl_vertex;
}

bool GouraudShader::fragment(Vector3f bar, TGAColor& color) {
  // interpolate intensity for the current pixel
  float intensity = varying_intensity * bar;
  // color = TGAColor(255, 255, 255) * intensity;
  Vector2f uv = varying_uv * bar;  // interpolate uv for the current pixel
  color = model->diffuse(uv) * intensity;
  return false;
}
