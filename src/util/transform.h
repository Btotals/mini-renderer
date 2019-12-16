#ifndef __TRANSFORM_H__
#define __TRANSFORM_H__

#include "./geometry.h"

Matrix translation(Vector3f v);
Matrix zoom(float factor);
// rotation_axis: 围绕 axis 轴旋转
Matrix rotation_x(float cosangle, float sinangle);
Matrix rotation_y(float cosangle, float sinangle);
Matrix rotation_z(float cosangle, float sinangle);

Matrix viewport(int x, int y, int w, int h, int depth);
Matrix lookat(Vector3f eye, Vector3f center, Vector3f up);

#endif
