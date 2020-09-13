#ifndef __TRANSFORM_H__
#define __TRANSFORM_H__

#include "./geo.h"

Matrix44f translation(const Vector3f& v);
Matrix44f zoom(float factor);
// rotation_axis: 围绕 axis 轴旋转
Matrix44f rotation_x(float radian);
Matrix44f rotation_y(float radian);
Matrix44f rotation_z(float radian);

#endif
