#include "./canvas/canvas.h"
#include "./lib/tga/tga_image.h"

#include "./model/model.h"
#include "./util/geometry.h"
#include "./util/transform.h"

#include <iostream>

void draw_line() {
  const int width = 100;
  const int height = 100;

  Canvas canvas(width, height, RGB);
  const TGAColor white = TGAColor(255, 255, 255, 255);
  const TGAColor red = TGAColor(255, 0, 0, 255);

  canvas.set_pixel_color(52, 41, red);
  canvas.line(0, 0, 100, 100, white);
  canvas.line(20, 13, 40, 80, red);
  canvas.line(80, 40, 13, 20, red);
}

void draw_triangle() {
  std::cout << "drawing triangles" << std::endl;

  const int width = 200;
  const int height = 200;

  const TGAColor red = TGAColor(255, 0, 0, 255);
  const TGAColor green = TGAColor(0, 255, 0, 255);
  const TGAColor blue = TGAColor(0, 0, 255, 255);

  Canvas canvas(width, height, RGB);

  Vector2i t0[3] = {Vector2i(10, 70), Vector2i(50, 160), Vector2i(70, 80)};
  Vector2i t1[3] = {Vector2i(180, 50), Vector2i(150, 1), Vector2i(70, 180)};
  Vector2i t2[3] = {Vector2i(180, 150), Vector2i(120, 160), Vector2i(130, 180)};
  // canvas.triangle(t0[0], t0[1], t0[2], red);
  canvas.triangle_barycentric_2d(t0[0], t0[1], t0[2], red);
  canvas.triangle_barycentric_2d(t1[0], t1[1], t1[2], green);
  canvas.triangle_barycentric_2d(t2[0], t2[1], t2[2], blue);

  canvas.write_tga_file("output.tga");
}

void draw_model(const char* file) {
  // const TGAColor color = TGAColor(255, 255, 255, 255);
  const int width = 800;
  const int height = 800;

  std::cout << file << std::endl;

  Model* model = new Model(file);

  Canvas canvas(width, height, RGB);

  // 把模型里面的点面对应的三条边画出来，不考虑重复情况
  // for (int i = 0; i < model->nfaces(); i++) {
  //   std::vector<int> face = model->face(i);
  //   for (int j = 0; j < 3; j++) {
  //     // 这里用到的坐标都是基于渲染空间 x,y,z ∈ [-1, 1]
  //     // 忽略 z 轴的情况下，只需要用 x,y 两个坐标即可
  //     // 因此，先 +1 变成正数，/2 之后可以映射到屏幕坐标
  //     Vector3f v0 = model->vert(face[j]);
  //     Vector3f v1 = model->vert(face[(j + 1) % 3]);
  //     int x0 = (v0.x + 1.) * width / 2.;
  //     int y0 = (v0.y + 1.) * height / 2.;
  //     int x1 = (v1.x + 1.) * width / 2.;
  //     int y1 = (v1.y + 1.) * height / 2.;
  //     canvas.line(x0, y0, x1, y1, color);
  //   }
  // }

  // 光照类型：平行光源，模拟日光情况
  Vector3f light_dir(0, 0, -1);

  // 没有开启 zBuffer 的深度测试，面片渲染结果会有 overlay
  // for (int i = 0, faces = model->nfaces(); i < faces; i++) {
  //   std::vector<int> face = model->face(i);
  //   Vector2i screen_coords[3];
  //   Vector3f world_coords[3];
  //   for (int j = 0; j < 3; j++) {
  //     Vector3f v = model->vert(face[j]);
  //     screen_coords[j] =
  //       Vector2i((v.x + 1.) * width / 2., (v.y + 1.) * height / 2.);
  //     // 模型使用的原始 3d 世界坐标
  //     world_coords[j] = v;
  //   }
  //   // 用向量积计算三角面片的单位法向量
  //   Vector3f n =
  //     (world_coords[2] - world_coords[0]) ^ (world_coords[1] -
  //     world_coords[0]);
  //   n.normalize();

  //   // 与光照方向向量做点积，越接近 1 亮度越高
  //   float intensity = n * light_dir; if (intensity > 0) {
  //     int luminance = intensity * 255;
  //     canvas.triangle(screen_coords[0], screen_coords[1], screen_coords[2],
  //                     TGAColor(luminance, luminance, luminance, 255));
  //   }
  // }

  canvas.enable_zbuffer();
  for (int i = 0; i < model->nfaces(); i++) {
    std::vector<int> face = model->face(i);
    Vector3f origin_coords[3];
    Vector3f coords[3];
    for (int j = 0; j < 3; j++) {
      Vector3f v = model->vert(face[j]);
      // 模型使用的原始 3d 世界坐标
      origin_coords[j] = v;
      coords[j] = Vector3f(int((v.x + 1.) * width / 2. + .5),
                           int((v.y + 1.) * height / 2. + .5), v.z);
    }

    Vector3f n = (origin_coords[2] - origin_coords[0]) ^
                 (origin_coords[1] - origin_coords[0]);
    // std::cout << n << std::endl;
    n.normalize();
    // 用模长为 1 的 z 方向向量与单位法向量做点积，结果越接近 1 意味着亮度越高
    float intensity = n * light_dir;
    if (intensity > 0) {
      int luminance = intensity * 255;
      canvas.triangle_barycentric_3d(
        coords[0], coords[1], coords[2],
        TGAColor(luminance, luminance, luminance, 255));
    }
  }
  std::cout << "draw finish" << std::endl;

  // 翻转 y 轴
  canvas.flip_vertically();
  canvas.write_tga_file("output.tga");
  delete model;
}

void draw_model_with_camera(char* file) {
  std::cout << file << std::endl;
  const int width = 800;
  const int height = 800;
  const int depth = 255;

  Vector3f light_dir = Vector3f(0, 0, 1).normalize();
  // 相机位置
  Vector3f position(0, 0, 3);
  // 原点位置
  Vector3f center(0, 0, 0);

  Model* model = new Model(file);

  Canvas canvas(width, height, RGB);
  // 开启深度测试
  canvas.enable_zbuffer();

  Matrix view = lookat(position, center, Vector3f(0, 1, 0));
  // 将整个视觉空间裁剪到屏幕
  // 左上角顶点 (width / 4, height / 4) 右下角 (width * 3 / 4, height * 3 / 4)
  Matrix view_port =
    viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4, depth);

  // 透视投影矩阵，根据相机距离原点的距离计算透视缩放分量为 -1/||center -
  // position||
  Matrix projection = Matrix::identity(4);
  projection[3][2] = -1.f / (position - center).norm();

  Matrix z = (view_port * projection * view);
  std::cerr << z << std::endl;
  for (int i = 0; i < model->nfaces(); i++) {
    std::vector<int> face = model->face(i);
    Vector3i screen_coords[3];
    Vector3f world_coords[3];
    float intensity[3];
    for (int j = 0; j < 3; j++) {
      Vector3f v = model->vert(face[j]);
      // 完整的 transformation chain: view_port * projection * view * model
      // 还需要有一个 model 矩阵，用于将 local space 变换到 world
      // 这个模型放到世界空间里面，对应的位置、缩放比例、旋转关系都由他决定
      // 这里直接把模型放原点，缩放为 1，不旋转，对应的 Model 矩阵是单位矩阵
      // 因此直接拿 v 来构造一个矩阵即可
      screen_coords[j] = Vector3f(z * Matrix(v));
      world_coords[j] = v;
      intensity[j] = model->norm(i, j) * light_dir;
    }
    canvas.triangle_gouraud(screen_coords[0], screen_coords[1],
                            screen_coords[2], intensity[0], intensity[1],
                            intensity[2]);
  }
  canvas.flip_vertically();  // i want to have the origin at the left bottom
                             // corner of the image
  canvas.write_tga_file("output.tga");
}

void calc_matrix() {
  Matrix m(3, 3);
  m[0] = Row({1, 1, -1});
  m[1] = Row({0, 1, 1});
  m[2] = Row({0, 0, 1});

  std::cout << m << std::endl;

  Matrix m_inv = m.inverse();
  std::cout << m_inv << std::endl;

  Matrix product = m * m_inv;
  std::cout << product << std::endl;
}

int main(int argc, char** argv) {
  // const TGAColor white = TGAColor(255, 255, 255, 255);

  // calc_matrix();

  if (argc == 2) {
    // draw_model(argv[1], white);
    draw_model_with_camera(argv[1]);
  } else {
    draw_triangle();
  }

  return 0;
}
