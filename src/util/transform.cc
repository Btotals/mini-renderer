#include "./transform.h"
#include <cmath>

/**
 * @brief 平移
 *
 * @param v 位移对应的向量
 * @return Matrix 变换矩阵
 */
Matrix44f translation(const Vector3f& v) {
  Matrix44f t = Matrix44f::identity();
  t[0][3] = v.x;
  t[1][3] = v.y;
  t[2][3] = v.z;
  return t;
}

Matrix44f zoom(float factor) {
  Matrix44f z = Matrix44f::identity();
  z[0][0] = z[1][1] = z[2][2] = factor;
  return z;
}

/**
 * @brief 绕 x 轴旋转
 *
 * @param radian 旋转的弧度角
 * @return Matrix 旋转矩阵
 */
Matrix44f rotation_x(float radian) {
  Matrix44f r = Matrix44f::identity();
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
Matrix44f rotation_y(float radian) {
  Matrix44f r = Matrix44f::identity();
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
Matrix44f rotation_z(float radian) {
  Matrix44f r = Matrix44f::identity();
  r[0][0] = r[1][1] = cos(radian);
  r[0][1] = -sin(radian);
  r[1][0] = sin(radian);

  return r;
}
