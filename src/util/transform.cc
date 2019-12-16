#include "./transform.h"
#include <cmath>

Matrix translation(Vector3f v) {
  Matrix t = Matrix::identity(4);
  t[0][3] = v.x;
  t[1][3] = v.y;
  t[2][3] = v.z;
  return t;
}

Matrix zoom(float factor) {
  Matrix z = Matrix::identity(4);
  z[0][0] = z[1][1] = z[2][2] = factor;
  return z;
}

/**
 * @brief 绕 x 轴旋转
 *
 * @param radian 旋转的弧度角
 * @return Matrix 旋转矩阵
 */
Matrix rotation_x(float radian) {
  Matrix r = Matrix::identity(4);
  r[1][1] = r[2][2] = cos(radian);
  r[1][2] = -sin(radian);
  r[2][1] = sin(radian);

  return r;
}

/**
 * @brief 绕 y 轴旋转
 *
 * @param radian 旋转的弧度角
 * @return Matrix 旋转矩阵
 */
Matrix rotation_y(float radian) {
  Matrix r = Matrix::identity(4);
  r[0][0] = r[2][2] = cos(radian);
  r[0][2] = -sin(radian);
  r[2][0] = sin(radian);

  return r;
}

/**
 * @brief 绕 z 轴旋转
 *
 * @param radian 旋转的弧度角
 * @return Matrix 旋转矩阵
 */
Matrix rotation_z(float radian) {
  Matrix r = Matrix::identity(4);
  r[0][0] = r[1][1] = cos(radian);
  r[0][1] = -sin(radian);
  r[1][0] = sin(radian);

  return r;
}

// 将整个 [-1, 1] * [-1, 1] * [-1, 1] 的单位空间
// 映射到 [0, w] * [0, h] * [0, d]
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
 * @return Matrix viewport 变换矩阵
 */
Matrix viewport(int x, int y, int w, int h, int depth) {
  Matrix m = Matrix::identity(4);
  m[0][3] = x + w / 2.f;
  m[1][3] = y + h / 2.f;
  m[2][3] = depth / 2.f;

  m[0][0] = w / 2.f;
  m[1][1] = h / 2.f;
  m[2][2] = depth / 2.f;
  return m;
}

/**
 * @brief 将世界坐标转化为用户视野坐标
 *
 * @param position 相机位置
 * @param center 场景原点
 * @param up 上向量，用于定义相机的正 x 轴
 * @return Matrix 相机/视觉 空间 (Camera/Eye Space)
 */
Matrix lookat(Vector3f position, Vector3f center, Vector3f up) {
  Vector3f z = (position - center).normalize();
  Vector3f x = (up ^ z).normalize();
  Vector3f y = (z ^ x).normalize();
  Matrix res = Matrix::identity(4);
  for (int i = 0; i < 3; i++) {
    res[0][i] = x[i];
    res[1][i] = y[i];
    res[2][i] = z[i];
    res[i][3] = -center[i];
  }
  return res;
}
