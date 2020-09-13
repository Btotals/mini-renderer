#include "./shader.h"
#include "../util/geo_util.h"
#include "../util/transform.h"
#include <cmath>
#include <cstdlib>
#include <limits>

const float PI = 2 * acos(0.f);

// IShader::~IShader() {}

Matrix44f view = Matrix44f::identity();
Matrix44f projection = Matrix44f::identity();
Matrix44f view_port = Matrix44f::identity();

Vector3f light_dir = Vector3f(1, 1, 1).normalize();
TGAColor light_color = TGAColor(255, 255, 255);

Vector3f camera_pos = Vector3f(0, 0, 0);

Matrix44f transition = translation(Vector3f(0, -1, 0)) * zoom(0.2);
// Matrix44f transition = Matrix44f::identity();

Matrix44f clip_matrix = Matrix44f::identity();
Matrix44f final_matrix = Matrix44f::identity();

Matrix44f normal_matrix = Matrix44f::identity();
Matrix44f normal_matrix_it = Matrix44f::identity();

float ambient = .1f;

void set_light(Vector3f dir, TGAColor c) {
  light_dir = dir.normalize();
  light_color = c;
}

/**
 * @brief 视口变换, 将 clip space 裁剪空间映射到 screen space 屏幕空间
 * 裁剪空间: [-1, 1] * [-1, 1] * [-1, 1]
 * 变换到 [x, x + w] * [y, y + h] * [0, d]
 *
 * @param x 空间坐标 x
 * @param y 空间坐标 y
 * @param w 立方体的宽度
 * @param h 立方体的高度
 * @return Matrix viewport 视口变换矩阵
 */
void viewport(int x, int y, int w, int h) {
  view_port[0][3] = x + w / 2.f;
  view_port[1][3] = y + h / 2.f;
  view_port[0][0] = w / 2.f;
  view_port[1][1] = h / 2.f;

  std::cout << "view_port matrix" << std::endl;
  std::cout << view_port << std::endl;

  normal_matrix = projection * view;
  normal_matrix_it = normal_matrix.invert_transpose();

  clip_matrix = normal_matrix * transition;
  final_matrix = view_port * clip_matrix;
}

/**
 * @brief 设置透视缩放分量
 *
 * @param coefficient 透视缩放分量
 * @return Matrix projection 投影矩阵
 */
void project(float coefficient) {
  projection[3][2] = coefficient;

  std::cout << "projection matrix" << std::endl;
  std::cout << projection << std::endl;

  normal_matrix = projection * view;
  normal_matrix_it = normal_matrix.invert_transpose();

  clip_matrix = normal_matrix * transition;
  final_matrix = view_port * clip_matrix;
}

/**
 * @brief 设置正交相机
 *
 * @param l 左坐标
 * @param r 右坐标
 * @param t 上坐标
 * @param b 下坐标
 * @param n 近平面距离
 * @param f 远平面距离
 */
void projectOrthographic(float l, float r, float b, float t, float n, float f) {
  projection[0][0] = 2.f / (r - l);
  projection[1][1] = 2.f / (t - b);
  // attention: 因为接口仿照 openGL 中的 glm::ortho, n & f 均为正数
  // 因此需要做一次取反
  projection[2][2] = -2.f / (n - f);

  projection[0][3] = -(r + l) / (r - l);
  projection[1][3] = -(t + b) / (t - b);
  projection[2][3] = -(n + f) / (n - f);

  std::cout << "projection matrix" << std::endl;
  std::cout << projection << std::endl;

  normal_matrix = projection * view;
  normal_matrix_it = normal_matrix.invert_transpose();

  clip_matrix = normal_matrix * transition;
  final_matrix = view_port * clip_matrix;
}

/**
 * @brief 设置透视相机
 *
 * @param fov field of view, 视角大小, 单位: degree
 * @param aspect 纵横比, 一般等于画布宽高之比
 * @param near 近平面距离
 * @param far 远平面距离
 */
void projectPerspective(float fov, float aspect, float near, float far) {
  Matrix44f c = Matrix44f::identity();
  c[0][0] = near;
  c[1][1] = near;
  c[2][2] = near + far;
  c[2][3] = -near * far;
  // attention: 因为我们使用右手坐标系, 与 NDC 相反, 因此这里要取反
  c[3][2] = -1;

  // 使用三角函数, 计算对应的 l r b t 值
  float half_fov_radian = (fov / 2) * PI / 180;
  float half_height = tan(half_fov_radian) * near;
  float half_width = half_height * aspect;

  projectOrthographic(-half_width, half_width, -half_height, half_height, near,
                      far);

  // 矩阵乘法满足结合律
  projection = projection * c;

  normal_matrix = projection * view;
  normal_matrix_it = normal_matrix.invert_transpose();

  clip_matrix = normal_matrix * transition;
  final_matrix = view_port * clip_matrix;
}

/**
 * @brief 将世界坐标转化为用户视野坐标
 * 前提: 如果对相机以及世界内所有物体 采用同一个线性变换 最后相机所见与原来一致
 * 本质上是在把任意放置+任意朝向的相机. 转换成
 * 位于原点(0, 0, 0) 面朝 z 轴负方向 (0, 0, -1) , 上方向 (0, 1, 0) 的标准相机
 * 同时 世界内其他物体也随之变换
 *
 * @param position 相机位置
 * @param direction 相机面对的正方向(z 轴)
 * @param up 用于定义相机的正上方向(y 轴)
 * @return Matrix 相机/视觉 空间 (Camera/Eye Space)
 */
void lookat(Vector3f position, Vector3f direction, Vector3f up) {
  Vector3f z = direction.normalize(), y = up.normalize(), x = z ^ y;
  Matrix44f mr = Matrix44f::identity(), mt = Matrix44f::identity();

  camera_pos = position;

  for (int i = 0; i < 3; i++) {
    // 这里对应旋转操作
    mr[0][i] = x[i];
    mr[1][i] = y[i];
    mr[2][i] = -z[i];

    // 这里对应平移操作
    mt[i][3] = -position[i];
  }

  view = mr * mt;
  std::cout << "view matrix" << std::endl;
  std::cout << view << std::endl;

  normal_matrix = projection * view;
  normal_matrix_it = normal_matrix.invert_transpose();

  clip_matrix = normal_matrix * transition;
  final_matrix = view_port * clip_matrix;
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
Vector3f barycentric(const Vector2f& a,
                     const Vector2f& b,
                     const Vector2f& c,
                     const Vector2f& p) {
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
 * @param zbuffer zbuffer float[w * h] to be draw
 */
void triangle(Vector4f* pts, IShader& shader, TGAImage& img, float* zbuffer) {
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
      // int frag_depth = max(0, min(255, int(z / w + .5)));
      float frag_depth = z / w;

      // z-buffer test
      int z_index = p.x + p.y * img.get_width();
      if (c.x < 0 || c.y < 0 || c.z < 0 || z_index >= img.get_size() ||
          zbuffer[z_index] > frag_depth) {
        continue;
      }

      bool discard = shader.fragment(c, color);
      if (!discard) {
        zbuffer[z_index] = frag_depth;
        img.set(p.x, p.y, color);
      }
    }
  }
}

/**
 * @brief draw a triangle with shader
 *
 * @param shader Shader shader with vertex and fragment unit
 * @param img img to be draw
 * @param zbuffer zbuffer float[w * h] to be draw
 */
void triangle(Matrix<4, 3, float>& varying_pts,
              IShader& shader,
              TGAImage& img,
              float* zbuffer) {
  // varying_pts 指裁剪空间中的三角形顶点坐标
  Matrix<3, 4, float> clip_pts = (view_port * varying_pts).transpose();
  // screen_pts 指变换到屏幕空间之后的坐标
  Matrix<3, 2, float> screen_pts;

  for (int i = 0; i < 3; i++) {
    screen_pts[i] = project<2>(clip_pts[i] / clip_pts[i][3]);
  }

  const float float_max = std::numeric_limits<float>::max();
  Vector2f bbox_min(float_max, float_max), bbox_max(-float_max, -float_max);
  Vector2f clamp(img.get_width() - 1, img.get_height() - 1);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      bbox_max[j] = min(clamp[j], max(bbox_max[j], screen_pts[i][j]));
      bbox_min[j] = max(0.f, min(bbox_min[j], screen_pts[i][j]));
    }
  }

  Vector2i p;
  TGAColor color;

  // foreach point in bbox, calculating its coordination of barycentric
  for (p.x = bbox_min.x; p.x < bbox_max.x; p.x++) {
    for (p.y = bbox_min.y; p.y < bbox_max.y; p.y++) {
      // 屏幕上三角形的重心坐标
      Vector3f bc_screen =
        barycentric(screen_pts[0], screen_pts[1], screen_pts[2], p);
      // 用屏幕空间的三角形重心坐标除以裁剪空间的 w 分量, 所得结果可以用于插值
      Vector3f bc_clip =
        Vector3f(bc_screen.x / clip_pts[0][3], bc_screen.y / clip_pts[1][3],
                 bc_screen.z / clip_pts[2][3]);
      // 重心坐标化
      bc_clip = bc_clip / (bc_clip.x + bc_clip.y + bc_clip.z);

      // 求出裁剪空间中的深度
      float frag_depth = varying_pts[2] * bc_clip;

      // z-buffer test
      int z_index = p.x + p.y * img.get_width();
      if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0 ||
          z_index >= img.get_size() || zbuffer[z_index] > frag_depth) {
        continue;
      }

      bool discard = shader.fragment(bc_clip, color);
      if (!discard) {
        zbuffer[z_index] = frag_depth;
        img.set(p.x, p.y, color);
      }
    }
  }
}

Vector4f FirstShader::vertex(int iface, int nthvert) {
  varying_uv.set_column(nthvert, model->uv(iface, nthvert));
  // 没有处理正切空间
  varying_normal.set_column(nthvert, model->norm(iface, nthvert));
  // 有处理
  // varying_normal.set_column(
  //   nthvert,
  //   project<3>(normal_matrix_it * embed<4>(model->norm(iface, nthvert),
  //   0.f)));

  // read the vertex from .obj file amd using cache matrix
  // Vector4f gl_vertex = view_port * projection * view * transition *
  //                      embed<4>(model->vert(iface, nthvert));

  // 这里先计算出世界空间中的位置
  Vector4f gl_vertex = clip_matrix * embed<4>(model->vert(iface, nthvert));
  varying_pts.set_column(nthvert, gl_vertex);
  // ndc_tri.set_column(nthvert, project<3>(gl_vertex / gl_vertex[3]));

  // 然后才是屏幕空间的位置
  return view_port * gl_vertex;
}

bool FirstShader::fragment(Vector3f bar, TGAColor& color) {
  Vector2f uv = varying_uv * bar;
  Vector3f normal = (varying_normal * bar).normalize();

  // Matrix<3, 3, float> A;
  // A[0] = ndc_tri.column(1) - ndc_tri.column(0);
  // A[1] = ndc_tri.column(2) - ndc_tri.column(0);
  // A[2] = normal;

  // Matrix<3, 3, float> AI = A.invert();

  // Vector3f i = AI * Vector3f(varying_uv[0][1] - varying_uv[0][0],
  //                            varying_uv[0][2] - varying_uv[0][0], 0);
  // Vector3f j = AI * Vector3f(varying_uv[1][1] - varying_uv[1][0],
  //                            varying_uv[1][2] - varying_uv[1][0], 0);

  // Matrix<3, 3, float> B;
  // B.set_column(0, i.normalize());
  // B.set_column(1, j.normalize());
  // B.set_column(2, normal);

  // Vector3f n = (B * model->norm(uv)).normalize();

  // float diffuse = max(0.f, -(n * light_dir));
  float diffuse = max(0.f, -(normal * light_dir));

  color = model->diffuse(uv) * light_color * diffuse;

  return false;
}

// bool FirstShader::fragment(Vector3f bar, TGAColor& color) {
//   Vector2f uv = varying_uv * bar;
//   Vector3f normal = varying_normal * bar;

//   Vector4f homo_pos = varying_pts * bar;
//   Vector3f pos = project<3>(homo_pos) * (1 / homo_pos[3]);
//   Vector3f camera = project<3>(projection * view * embed<4>(camera_pos));

//   Vector3f view_dir = (camera - pos).normalize();
//   Vector3f reflect_dir = reflect(light_dir, normal).normalize();

//   // float specular = pow(max(0.f, view_dir * reflect_dir), 16);
//   float diffuse = max(0.f, -(normal.normalize() * light_dir));

//   color = model->diffuse(uv) * light_color * diffuse;
//   // color =
//   //   model->diffuse(uv) * light_color * (ambient + diffuse + specular *
//   0.6);

//   return false;
// }

Vector4f GouraudShader::vertex(int iface, int nthvert) {
  varying_uv.set_column(nthvert, model->uv(iface, nthvert));
  // read the vertex from .obj file amd using cache matrix
  Vector4f gl_vertex = final_matrix * embed<4>(model->vert(iface, nthvert));

  // get diffuse lighting intensity
  varying_intensity[nthvert] =
    max(0.f, model->norm(iface, nthvert) * light_dir);

  return gl_vertex;
}

bool GouraudShader::fragment(Vector3f bar, TGAColor& color) {
  // color = TGAColor(255, 255, 255) * intensity;
  // interpolate intensity for current pixel
  float intensity = varying_intensity * bar;
  Vector2f uv = varying_uv * bar;  // interpolate uv for current pixel

  // Vector3f n =
  //   project<3>(normal_matrix_it * embed<4>(model->norm(uv))).normalize();
  // Vector3f l = project<3>(normal_matrix * embed<4>(light_dir)).normalize();
  // float intensity = max(0.f, n * l);

  color = model->diffuse(uv) * intensity;

  return false;
}

// bool GouraudShader::fragment(Vector3f bar, TGAColor& color) {
//   // color = TGAColor(255, 255, 255) * intensity;
//   // interpolate intensity for current pixel
//   // float intensity = varying_intensity * bar;
//   Vector2f uv = varying_uv * bar;  // interpolate uv for current pixel

//   Vector3f n =
//     project<3>(normal_matrix_it * embed<4>(model->norm(uv))).normalize();
//   Vector3f l = project<3>(normal_matrix * embed<4>(light_dir)).normalize();
//   Vector3f r = (n * (n * l * 2.f) - l).normalize();  // reflected light

//   float spec = pow(max(r.z, 0.0f), model->specular(uv));
//   float diff = max(0.f, n * l);

//   // float intensity = max(0.f, n * l);

//   // color = model->diffuse(uv) * intensity;
//   // color = model->diffuse(uv) * 2;
//   TGAColor c = model->diffuse(uv);

//   for (int i = 0; i < 3; i++) {
//     color[i] = min<float>(5 + c[i] * (diff + .6 * spec), 255);
//   }

//   return false;
// }

Vector4f PhongShader::vertex(int iface, int nthvert) {
  varying_uv.set_column(nthvert, model->uv(iface, nthvert));

  // with homogeneous value 0 means it is a vector rather than point
  Vector4f origin_normal = embed<4>(model->norm(iface, nthvert), 0.f);

  // Phong need interpolate normal, which should be translate by normal matrix
  Vector3f normal = project<3>(normal_matrix_it * origin_normal);
  varying_nrm.set_column(nthvert, normal);

  // read the vertex from .obj file amd using cache matrix
  Vector4f gl_vertex = final_matrix * embed<4>(model->vert(iface, nthvert));

  return gl_vertex;
}

bool PhongShader::fragment(Vector3f bar, TGAColor& color) {
  Vector3f bn = (varying_nrm * bar).normalize();
  Vector2f uv = varying_uv * bar;

  float diff = max(0.f, -(bn * light_dir));
  color = model->diffuse(uv) * diff;
  // color = model->diffuse(uv);
  return false;
}
